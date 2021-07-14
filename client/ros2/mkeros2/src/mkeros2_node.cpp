/*
 * mkeros2_node
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Jigar Patel, jigar@magik-eye.com
 */

// ============================================================================
// ROS2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>

// ============================================================================
// System

#include <iostream>
#include <thread>
#include <chrono>

// ============================================================================
// mkeros2_node

#include "cliparser.h"
#include "device_info.h"
#include "device_discovery.h"
#include "publisher.h"

// ============================================================================
// Namespaces
using namespace std::chrono_literals;

// ============================================================================
// Globals

std::shared_ptr<MkEPointCloudPublisher> publisher;
DeviceDiscovery device_discovery;
DeviceInfo device_info;

// ============================================================================
// ============================================================================
// ROS callbacks

/**
    Start service callback.
    @param std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res
    @return bool - Success/Failure
    @brief
    To Start publishing on PCD Topic
*/
bool start_service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!publisher)
    return true;

  auto uid = device_info.getUnitId();
  auto start_service = device_info.getStartServiceName();

  try
    {
      if (!uid.empty())
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Looking for device: %s", uid.c_str());
          device_discovery.updateDeviceList();

          DeviceInfo dinfo;
          if (!device_discovery.validateDevice(uid, dinfo))
            throw std::runtime_error("Cannot find device: " + uid);

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Found device: %s: %s", dinfo.getUnitIdCStr(),
                   dinfo.getIpAddrCStr());
          publisher->setDeviceInfo(dinfo);
        }

      publisher->startPublishing();

      res->success = true;
      res->message = "";

      auto status_msg = "Started point cloud publishing at " + start_service;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s", status_msg.c_str());
    }
  catch (const std::exception& e)
    {
      res->success = false;
      res->message = e.what();

      auto status_msg = "Failed to start point cloud publishing at " +
                        start_service + ": " + e.what();

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s", status_msg.c_str());
      return false;
    }

  return true;
}

// ============================================================================
/**
    Stop service callback.
    @param std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res
    @return bool - Success/Failure
    @brief
    To Stop publishing on PCD Topic
*/
bool stop_service_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!publisher)
    return true;

  auto uid = device_info.getUnitId();
  auto stop_service = device_info.getStopServiceName();

  try
    {
      publisher->stopPublishing();

      res->success = true;
      res->message = "";

      auto status_msg = "Stopped point cloud publishing at " + stop_service;

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s", status_msg.c_str());
    }
  catch (const std::exception& e)
    {
      res->success = false;
      res->message = e.what();

      auto status_msg = "Failed to stop point cloud publishing, " +
                        stop_service + ": " + e.what();

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s", status_msg.c_str());
      return false;
    }

  return true;
}

// ============================================================================
// ============================================================================
// ROS node initialization

void launch_node(const DeviceInfo& dinfo)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Launching node: %s", dinfo.getNodeName().c_str());
  
  publisher = std::make_shared<MkEPointCloudPublisher>(dinfo.getNodeName(), dinfo);

  // Start service server
  std::string start_service = dinfo.getStartServiceName();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting service: %s", start_service.c_str());
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_start =
    publisher->create_service<std_srvs::srv::Trigger>(start_service, &start_service_callback);

  // Stop service server
  std::string stop_service = dinfo.getStopServiceName();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting service: %s", stop_service.c_str());
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_stop =
    publisher->create_service<std_srvs::srv::Trigger>(stop_service, &stop_service_callback);

  rclcpp::spin(publisher);
}

// ============================================================================

int start_publishing(const DeviceInfo& dinfo)
{
  auto n = rclcpp::Node::make_shared(dinfo.getStartServiceNodeName());
  auto start_service = dinfo.getStartServiceName();

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_start =
          n->create_client<std_srvs::srv::Trigger>(start_service);
  
  auto srv = std::make_shared<std_srvs::srv::Trigger::Request>();

  while (!client_start->wait_for_service(1s))
    {
      if (!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return 0;
        }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Calling service: %s", start_service.c_str());
  
  auto result = client_start->async_send_request(srv);
  
  // Wait for the result. 
  if (rclcpp::spin_until_future_complete(n, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service called successfully: %s", start_service.c_str());
      return 0;
    } 
  else 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed, %s: %s", start_service.c_str(),
                                                result.get()->message.c_str());
      return 1;
    }

  return 0;
}

// ============================================================================

int stop_publishing(const DeviceInfo& dinfo)
{
  auto n = rclcpp::Node::make_shared(dinfo.getStopServiceNodeName());
  
  auto stop_service = dinfo.getStopServiceName();

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_stop =
          n->create_client<std_srvs::srv::Trigger>(stop_service);
  
  auto srv = std::make_shared<std_srvs::srv::Trigger::Request>();

  while (!client_stop->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) 
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return 0;
        }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Calling service: %s", stop_service.c_str());
  
  auto result = client_stop->async_send_request(srv);
  
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(n, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service called successfully: %s", stop_service.c_str());
    } 
    else 
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service call failed, %s: %s", stop_service.c_str(),
                result.get()->message.c_str());
      return 1;
    }
  
  return 0;
}

// ============================================================================
// ============================================================================
// CliParser

class CliParser : public mke::ros::CliParserService
{
 public:
  bool help;
  bool discover;
  bool start;
  bool stop;
  bool launch;
  bool version;
  int timeout;

  std::string device{};
  std::string alias{};

  CliParser()
  {
    registerBln(help, "help", "Print this help message", false);
    registerBln(version, "version", "Output version string", false);
    registerBln(start, "start",
                "Start node services (if applicable), "
                "and point cloud publishing topic",
                false);
    registerBln(stop, "stop", "Stop point cloud publiblishing topic", false);
    registerBln(launch, "launch", "Launch node and start its services", false);
    registerBln(discover, "discover",
                "Perform the local network sensor discovery procedure "
                "and output a list of available sensor",
                false);

    registerStr(device, "device", "IP address or Unit ID of an MkE device", "");
    registerStr(alias, "alias", "Device name alias", "");

    registerInt(timeout, "timeout", "Device discovery timeout [s]", 1);

    initConfig();
  }
};

// ============================================================================
// ============================================================================
// Main

int main(int argc, char* argv[])
{
  try
    {
      rclcpp::init(argc, argv);
      auto param_node = std::make_shared<rclcpp::Node>("MKEROS2_NODE");
      
      std::string device_param;
      param_node->declare_parameter<std::string>("device", "");
      param_node->get_parameter("device",device_param);
      
      if(device_param!="")
        {
          if (DeviceDiscovery::isIpAddress(device_param))
            {
              device_info.setIpAddr(device_param);
            }
          else
            {
              device_info.setUnitId(device_param);
            }

          std::string alias_param;
          param_node->declare_parameter<std::string>("alias","");
          param_node->get_parameter("alias",alias_param);

          if(alias_param!="")
            {
              device_info.setAlias(alias_param);
            }
          else
            {
              device_info.setAlias();
            }
          launch_node(device_info);
          return 0;
        }

      CliParser cli;
      cli.parseFromCmdLine(argc, argv);

      if (cli.help)
        {
          std::cout << "This is mkeros2_node v" << MKEROS2_VERSION 
                    << ", (c) MagikEye 2020-2021";
          std::cout << std::endl;

          cli.getCommandLineHelp(std::cout);
          return 0;
        }

      if (cli.version)
        {
          std::cout << MKEROS2_VERSION << std::endl;
        }

      device_discovery.setTimeout(static_cast<unsigned int>(cli.timeout));

      if (cli.discover)
        {
          if (cli.device.empty())
            {
              // Search for devices on local network
              device_discovery.updateDeviceList();

              // Output found devices
              for (const auto& ss : device_discovery.getDeviceList())
                {
                  std::cout << ss.first << ":" << ss.second << "\n";
                }

              return 0;
            }
          else
            {
              // Search for devices on local network
              device_discovery.updateDeviceList();

              // Check the availability of the cli.device only
              return device_discovery.validateDevice(cli.device) ? 0 : 1;
            }
        }

      if (DeviceDiscovery::isIpAddress(cli.device))
        {
          device_info.setIpAddr(cli.device);
        }
      else
        {
          device_info.setUnitId(cli.device);
        }

      device_info.setAlias(cli.alias);

      // Perform launch/start/stop action
      if (cli.launch)
        {
          if (cli.device.empty())
            {
              throw std::runtime_error("Please set the '--device' option");
            }
            
          launch_node(device_info);
        }
      else if (cli.start)
        {
          if (device_info.getName().empty())
          {
              throw std::runtime_error("Please set the '--device' or '--alias' option");
          }
          
          return start_publishing(device_info);
        }
      else if (cli.stop)
        {
          if (device_info.getName().empty())
          {
              throw std::runtime_error("Please set the '--device' or '--alias' option");
          }

          return stop_publishing(device_info);
        }
    }
  catch (std::exception& e)
    {
      std::cerr << "mkeros2_node fatal error: " << e.what() << std::endl;
      return 1;
    }

  rclcpp::shutdown();
  return 0;
}