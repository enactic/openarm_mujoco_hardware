// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <openarm_mujoco_hardware/openarm_mujoco_hardware.hpp>

namespace openarm_mujoco_hardware {

hardware_interface::CallbackReturn MujocoHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto mode = info.hardware_parameters.find("command_interface");
  if (mode != info.hardware_parameters.end()) command_interface_ = mode->second;
  if (command_interface_ != "effort" && command_interface_ != "position")
    return CallbackReturn::ERROR;
  // Legacy effort mode retains its eight PD gains; position mode has no DOF
  // limit.
  if (command_interface_ == "effort" && info_.joints.size() > TOTAL_DOF)
    return CallbackReturn::ERROR;
  const auto timeout = info.hardware_parameters.find("state_timeout");
  if (timeout != info.hardware_parameters.end())
    state_timeout_ = std::stod(timeout->second);
  if (!std::isfinite(state_timeout_) || state_timeout_ <= 0.0)
    return CallbackReturn::ERROR;

  const auto wait = info.hardware_parameters.find("wait_for_state");
  if (wait != info.hardware_parameters.end()) {
    auto value = wait->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    wait_for_state_ = value == "true" || value == "1";
  }
  const auto connection_timeout =
      info.hardware_parameters.find("connection_timeout");
  if (connection_timeout != info.hardware_parameters.end())
    connection_timeout_ = std::stod(connection_timeout->second);
  if (!std::isfinite(connection_timeout_) || connection_timeout_ <= 0.0)
    return CallbackReturn::ERROR;

  if (info.hardware_parameters.find("websocket_port") !=
      info.hardware_parameters.end()) {
    const double val = std::stoi(info.hardware_parameters.at("websocket_port"));
    if (val <= 0 || val > 65535) {
      return hardware_interface::CallbackReturn::FAILURE;
    }
    websocket_port_ = val;
  } else {
    websocket_port_ = kDefaultWebsocketPort;
  }
  std::cerr << "websocket port: " << websocket_port_ << std::endl;
  address_ = boost::asio::ip::make_address("0.0.0.0");
  endpoint_ = boost::asio::ip::tcp::endpoint(address_, websocket_port_);

  // allocate space for joint states
  const size_t DOF = info_.joints.size();

  qpos_.resize(DOF, 0.0);
  qvel_.resize(DOF, 0.0);
  qtau_.resize(DOF, 0.0);

  cmd_qpos_.resize(DOF, 0.0);
  cmd_qvel_.resize(DOF, 0.0);
  cmd_qtau_ff_.resize(DOF, 0.0);
  received_pos_.resize(DOF, 0.0);
  received_vel_.resize(DOF, 0.0);
  received_tau_.resize(DOF, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

void MujocoHardware::start_accept() {
  acceptor_.async_accept(
      [this](boost::beast::error_code ec, boost::asio::ip::tcp::socket socket) {
        if (ec) {
          if (ec != boost::asio::error::operation_aborted)
            std::cerr << "error accepting connection: " << ec.message()
                      << std::endl;
          return;
        }
        std::cout << "new connection accepted." << std::endl;
        {
          std::lock_guard<std::mutex> lock(session_mutex_);
          if (!ws_session_) {
            ws_session_ = WebSocketSession::create(std::move(socket), this);
            ws_session_->run();
          } else {
            boost::beast::error_code ignored;
            socket.close(ignored);  // One simulator owns a hardware system.
          }
        }
        this->start_accept();
      });
}

hardware_interface::CallbackReturn MujocoHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  ioc_.restart();
  state_received_ = false;
  commands_initialized_ = false;
  boost::beast::error_code ec;

  if (acceptor_.is_open()) {
    return hardware_interface::CallbackReturn::FAILURE;
  } else {
    acceptor_.open(endpoint_.protocol(), ec);
    if (ec) {
      throw std::runtime_error("open error: " + ec.message());
    }
  }

  acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
  if (ec) {
    throw std::runtime_error("enable address reuse error: " + ec.message());
  }

  acceptor_.bind(endpoint_, ec);
  if (ec) {
    throw std::runtime_error("bind error: " + ec.message());
  }

  acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
  if (ec) {
    throw std::runtime_error("listen error: " + ec.message());
  }

  start_accept();

  ioc_thread_ = std::thread([this]() {
    try {
      ioc_.run();
    } catch (const std::exception &e) {
      std::cerr << "error in io_context thread: " << e.what() << std::endl;
    }
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_cmd_torque();
  stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (wait_for_state_) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    const bool ready = state_cv_.wait_for(
        lock, std::chrono::duration<double>(connection_timeout_), [this] {
          return state_received_ &&
                 std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - state_time_)
                         .count() <= state_timeout_;
        });
    if (!ready) {
      std::cerr
          << "No fresh Web state received before hardware activation timeout"
          << std::endl;
      return CallbackReturn::FAILURE;
    }
    std::copy(received_pos_.begin(), received_pos_.end(), qpos_.begin());
    std::copy(received_vel_.begin(), received_vel_.end(), qvel_.begin());
    std::copy(received_tau_.begin(), received_tau_.end(), qtau_.begin());
    std::copy(qpos_.begin(), qpos_.end(), cmd_qpos_.begin());
    commands_initialized_ = true;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

MujocoHardware::~MujocoHardware() { stop(); }

void MujocoHardware::stop() {
  ioc_.stop();
  if (ioc_thread_.joinable()) ioc_thread_.join();
  boost::beast::error_code ec;
  acceptor_.close(ec);
  std::lock_guard<std::mutex> lock(session_mutex_);
  ws_session_.reset();
}

void MujocoHardware::send_json(const nlohmann::json &message) {
  std::lock_guard<std::mutex> lock(session_mutex_);
  if (ws_session_) ws_session_->send_json(message);
}

void MujocoHardware::clear_cmd_torque() {
  nlohmann::json message = {{"enabled", false},
                            {"command_interface", command_interface_}};
  // Zero torque remains correct for legacy clients; zero position is not a
  // stop.
  if (command_interface_ == "effort") {
    for (const auto &joint : info_.joints) message["cmd"][joint.name] = 0.0;
  }
  send_json(message);
}

hardware_interface::CallbackReturn MujocoHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_cmd_torque();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_cmd_torque();
  stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>
MujocoHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < qpos_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_POSITION,
        &qpos_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &qvel_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT, &qtau_[i]));
  }
  return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
MujocoHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < qpos_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_POSITION,
        &cmd_qpos_[i]));
    if (command_interface_ == "position") continue;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &cmd_qvel_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT,
        &cmd_qtau_ff_[i]));
  }
  return command_interfaces;
}
hardware_interface::return_type MujocoHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_received_) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      state_time_)
            .count() > state_timeout_) {
      clear_cmd_torque();
      return hardware_interface::return_type::ERROR;
    }
    // Copy into the exported storage; never write that storage from the IO
    // thread.
    std::copy(received_pos_.begin(), received_pos_.end(), qpos_.begin());
    std::copy(received_vel_.begin(), received_vel_.end(), qvel_.begin());
    std::copy(received_tau_.begin(), received_tau_.end(), qtau_.begin());
    if (!commands_initialized_) {
      std::copy(qpos_.begin(), qpos_.end(), cmd_qpos_.begin());
      commands_initialized_ = true;
    }
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MujocoHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!commands_initialized_) return hardware_interface::return_type::OK;
  nlohmann::json message = {{"command_interface", command_interface_},
                            {"enabled", true}};
  for (size_t i = 0; i < cmd_qpos_.size(); ++i) {
    const double value = command_interface_ == "position"
                             ? cmd_qpos_[i]
                             : KP_[i] * (cmd_qpos_[i] - qpos_[i]) +
                                   KD_[i] * (cmd_qvel_[i] - qvel_[i]) +
                                   cmd_qtau_ff_[i];
    if (!std::isfinite(value)) return hardware_interface::return_type::ERROR;
    message["cmd"][info_.joints[i].name] = value;
  }
  send_json(message);
  return hardware_interface::return_type::OK;
}

WebSocketSession::WebSocketSession(boost::asio::ip::tcp::socket socket,
                                   MujocoHardware *hw)
    : ws_(std::move(socket)), hw_(hw), write_in_progress_(false) {}

std::shared_ptr<WebSocketSession> WebSocketSession::create(
    boost::asio::ip::tcp::socket socket, MujocoHardware *hw) {
  return std::make_shared<WebSocketSession>(std::move(socket), hw);
}

void WebSocketSession::send_json(const nlohmann::json &j) {
  std::shared_ptr<std::string> msg = std::make_shared<std::string>(j.dump());
  const bool command = j.contains("cmd");
  boost::asio::post(ws_.get_executor(),
                    [self = shared_from_this(), msg, command] {
                      // Do not replay an accumulated trajectory after a slow
                      // connection.
                      if (command) self->send_queue_.clear();
                      self->send_queue_.push_back(msg);
                      if (!self->write_in_progress_) {
                        self->flush();
                      }
                    });
}

void WebSocketSession::flush() {
  if (send_queue_.empty()) {
    write_in_progress_ = false;
    return;
  }

  write_in_progress_ = true;
  auto msg = send_queue_.front();
  send_queue_.pop_front();

  ws_.async_write(boost::asio::buffer(*msg),
                  [self = shared_from_this(), msg](boost::beast::error_code ec,
                                                   std::size_t) {
                    if (ec) {
                      std::cerr << "send error: " << ec.message() << std::endl;
                    }
                    self->write_in_progress_ = false;
                    self->flush();
                  });
}

void WebSocketSession::run() { do_handshake(); }

void WebSocketSession::do_handshake() {
  ws_.set_option(boost::beast::websocket::stream_base::timeout::suggested(
      boost::beast::role_type::server));
  ws_.async_accept(boost::beast::bind_front_handler(
      &WebSocketSession::on_accept, shared_from_this()));
}

void WebSocketSession::on_accept(boost::beast::error_code ec) {
  if (ec) {
    std::cerr << "handshake failed: " << ec.message() << std::endl;
    std::lock_guard<std::mutex> lock(hw_->session_mutex_);
    if (hw_->ws_session_.get() == this) hw_->ws_session_.reset();
    return;
  }
  do_read();
}

void WebSocketSession::do_read() {
  ws_.async_read(buffer_, boost::beast::bind_front_handler(
                              &WebSocketSession::on_read, shared_from_this()));
}

void WebSocketSession::on_read(boost::beast::error_code ec,
                               std::size_t bytes_transferred) {
  if (ec) {
    std::cerr << "read error: " << ec.message() << std::endl;
    std::lock_guard<std::mutex> lock(hw_->session_mutex_);
    if (hw_->ws_session_.get() == this) hw_->ws_session_.reset();
    return;
  }
  std::string data = boost::beast::buffers_to_string(buffer_.data());
  {
    std::lock_guard<std::mutex> lock(hw_->state_mutex_);
    try {
      nlohmann::json j = nlohmann::json::parse(data);
      if (j.value("hello", false)) {
        nlohmann::json hello = {{"command_interface", hw_->command_interface_},
                                {"joints", nlohmann::json::array()}};
        for (const auto &joint : hw_->info_.joints)
          hello["joints"].push_back(joint.name);
        const auto scene = hw_->info_.hardware_parameters.find("scene_group");
        if (scene != hw_->info_.hardware_parameters.end()) hello["scene_group"] = scene->second;
        send_json(hello);
      }
      if (j.contains("state")) {
        const auto &state = j.at("state");
        std::vector<double> pos, vel, tau;
        for (const auto &joint : hw_->info_.joints) {
          const auto &value = state.at(joint.name);
          pos.push_back(value.at("qpos").get<double>());
          vel.push_back(value.at("qvel").get<double>());
          tau.push_back(value.value("qtau", 0.0));
          if (!std::isfinite(pos.back()) || !std::isfinite(vel.back()) ||
              !std::isfinite(tau.back()))
            return;
        }
        hw_->received_pos_ = std::move(pos);
        hw_->received_vel_ = std::move(vel);
        hw_->received_tau_ = std::move(tau);
        hw_->state_received_ = true;
        hw_->state_time_ = std::chrono::steady_clock::now();
        hw_->state_cv_.notify_all();
      }
    } catch (const nlohmann::json::exception &e) {
      std::cerr << "json parse error: " << e.what() << std::endl;
    }
  };

  buffer_.consume(bytes_transferred);
  do_read();
}

};  // namespace openarm_mujoco_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_mujoco_hardware::MujocoHardware,
                       hardware_interface::SystemInterface)