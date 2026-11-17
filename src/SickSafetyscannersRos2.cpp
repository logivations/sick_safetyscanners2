// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2020, SICK AG, Waldkirch
*  Copyright (C) 2020, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file SickSafetyscannersRos.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2020-12-08
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners2/SickSafetyscannersRos2.h>

namespace sick {

SickSafetyscannersRos2::SickSafetyscannersRos2(const rclcpp::NodeOptions& options):
    Node("SickSafetyscannersRos2", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing SickSafetyscannersRos2 Node");

  // read parameters!
  initializeParameters(*this);
  loadParameters(*this);
  // not in initializeParameters: the lifecycle node has no reconnect watchdog
  const double reconnect_timeout =
      this->declare_parameter<double>("reconnect_timeout", 2.0);
  RCLCPP_INFO(this->get_logger(), "reconnect_timeout: %f", reconnect_timeout);
  m_reconnect_timeout = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(reconnect_timeout));

  // init publishers and services
  // set QOS profile for publishers explicitly
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data).keep_last(1);
  m_laser_scan_publisher =
      this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
  m_extended_laser_scan_publisher = this->create_publisher<
      sick_safetyscanners2_interfaces::msg::ExtendedLaserScan>("extended_scan",
                                                               qos);
  m_output_paths_publisher =
      this->create_publisher<sick_safetyscanners2_interfaces::msg::OutputPaths>(
          "output_paths", qos);
  m_raw_data_publisher = this->create_publisher<
      sick_safetyscanners2_interfaces::msg::RawMicroScanData>("raw_data", qos);

  m_field_data_service =
      this->create_service<sick_safetyscanners2_interfaces::srv::FieldData>(
          "field_data",
          std::bind(&SickSafetyscannersRos2::getFieldData, this,
                    std::placeholders::_1, std::placeholders::_2));

  m_status_overview_service = this->create_service<
      sick_safetyscanners2_interfaces::srv::StatusOverview>(
      "status_overview",
      std::bind(&SickSafetyscannersRos2::getStatusOverview, this,
                std::placeholders::_1, std::placeholders::_2));

  // Dynamic Parameter Change client
  m_param_callback = add_on_set_parameters_callback(
      std::bind(&SickSafetyscannersRos2::parametersCallback, this,
                std::placeholders::_1));

  // Without the watchdog keep the upstream behaviour: an unreachable sensor
  // fails the node construction.
  if (reconnect_timeout > 0.0) {
    connectSensor();
  } else {
    setupCommunication(std::bind(&SickSafetyscannersRos2::receiveUDPPaket,
                                 this, std::placeholders::_1));
  }

  startCommunication(this, m_laser_scan_publisher);
  touchLastDataTime();

  if (reconnect_timeout > 0.0) {
    m_reconnect_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SickSafetyscannersRos2::checkConnection, this));
  }

  RCLCPP_INFO(this->get_logger(), "Node Configured and running");
}

void SickSafetyscannersRos2::touchLastDataTime() {
  m_last_data_time = std::chrono::steady_clock::now().time_since_epoch().count();
}

bool SickSafetyscannersRos2::connectSensor() {
  try {
    // Destroy the old device first: joins its IO thread (so no callback runs
    // while the message creator is replaced) and drops a CoLa2 session that
    // may be stuck on a socket of the rebooted sensor.
    m_device.reset();
    // The sensor may have rebooted: its clock restarted, the mapping is void.
    resetDeviceTime();
    setupCommunication(std::bind(&SickSafetyscannersRos2::receiveUDPPaket,
                                 this, std::placeholders::_1));
    return true;
  } catch (const std::exception &e) {
    m_device.reset();
    RCLCPP_WARN(get_logger(), "Could not connect to sensor %s: %s",
                m_config.m_sensor_ip.to_string().c_str(), e.what());
    return false;
  }
}

void SickSafetyscannersRos2::checkConnection() {
  const auto last_data = std::chrono::steady_clock::time_point(
      std::chrono::steady_clock::duration(m_last_data_time.load()));
  if (std::chrono::steady_clock::now() - last_data < m_reconnect_timeout) {
    return;
  }

  RCLCPP_WARN(get_logger(),
              "No data from sensor %s for %.1f s, reconnecting",
              m_config.m_sensor_ip.to_string().c_str(),
              std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            last_data)
                  .count());
  // An attempt against a booting sensor can block for the CoLa2 timeouts;
  // the next one starts reconnect_timeout after this one ended.
  if (connectSensor()) {
    try {
      m_device->run();
      m_device->changeSensorSettings(m_config.m_communications_settings);
      RCLCPP_INFO(get_logger(), "Reconnected to sensor %s",
                  m_config.m_sensor_ip.to_string().c_str());
    } catch (const std::exception &e) {
      m_device.reset();
      RCLCPP_WARN(get_logger(), "Could not configure sensor %s: %s",
                  m_config.m_sensor_ip.to_string().c_str(), e.what());
    }
  }
  touchLastDataTime();
}


void SickSafetyscannersRos2::receiveUDPPaket(
    const sick::datastructure::Data &data) {
  touchLastDataTime();

  if (!m_config.m_msg_creator) {
    RCLCPP_WARN(get_logger(), "Received UDPP packet before all objects were "
                              "instantiated, ignoring this packet.");
    return;
  }

  if (!data.getMeasurementDataPtr()->isEmpty() &&
      !data.getDerivedValuesPtr()->isEmpty()) {
    // Every scan goes through the device time mapping, also while nobody
    // subscribes, so the offset estimate is warm when a subscriber appears.
    const rclcpp::Time stamp = scanStamp(data, this->now());
    if(m_diagnosed_laser_scan_publisher->getPublisher()->get_subscription_count() > 0){
      auto scan = m_config.m_msg_creator->createLaserScanMsg(data, stamp);
      m_diagnosed_laser_scan_publisher->publish(std::move(scan));
    }
    
    if(m_extended_laser_scan_publisher->get_subscription_count() > 0){
      auto extended_scan =
          m_config.m_msg_creator->createExtendedLaserScanMsg(data, stamp);
      m_extended_laser_scan_publisher->publish(std::move(extended_scan));
    }
    if(m_output_paths_publisher->get_subscription_count() > 0){
      auto output_paths = m_config.m_msg_creator->createOutputPathsMsg(data);
      m_output_paths_publisher->publish(std::move(output_paths));
    }
  }

  auto raw_msg = m_config.m_msg_creator->createRawDataMsg(data);
  m_last_raw_msg = *raw_msg;
  if(m_raw_data_publisher->get_subscription_count() > 0) {
    m_raw_data_publisher->publish(std::move(raw_msg));
  }
}
} // namespace sick

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sick::SickSafetyscannersRos2)
