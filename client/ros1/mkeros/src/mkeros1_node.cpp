/*
 * mkeros1_node
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Jigar Patel, jigar@magik-eye.com
 *         Gunjan Sethi, gunjan@magik-eye.com
 *         Jan Heller, jan@magik-eye.com
 */

// ============================================================================
// ROS

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "std_srvs/Trigger.h"

// ============================================================================
// System

#include <iostream>
#include <thread>

// ============================================================================
// mkeros1_node

#include "cliparser.h"
#include "device_info.h"
#include "device_discovery.h"
#include "publisher.h"

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
bool start_service_callback(std_srvs::Trigger::Request& req,
                            std_srvs::Trigger::Response& res)
{
  if (!publisher)
    return true;

  auto uid = device_info.getUnitId();
  auto start_service = device_info.getStartServiceName();

  try
    {
      if (!uid.empty())
        {
          ROS_INFO("Looking for device: %s", uid.c_str());
          device_discovery.updateDeviceList();

          DeviceInfo dinfo;
          if (!device_discovery.validateDevice(uid, dinfo))
            throw std::runtime_error("Cannot find device: " + uid);

          ROS_INFO("Found device: %s: %s", dinfo.getUnitIdCStr(),
                   dinfo.getIpAddrCStr());
          publisher->setDeviceInfo(dinfo);
        }

      publisher->startPublishing();

      auto status_msg = "Started point cloud publishing at " + start_service;
      
      res.success = true;
      res.message = status_msg;

      ROS_INFO("%s", status_msg.c_str());
    }
  catch (const std::exception& e)
    {
      res.success = false;
      res.message = e.what();

      auto status_msg = "Failed to start point cloud publishing at " +
                        start_service + ": " + e.what();

      ROS_INFO("%s", status_msg.c_str());
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
bool stop_service_callback(std_srvs::Trigger::Request& req,
                           std_srvs::Trigger::Response& res)
{
  if (!publisher)
    return true;

  auto uid = device_info.getUnitId();
  auto stop_service = device_info.getStopServiceName();

  try
    {
      publisher->stopPublishing();

      auto status_msg = "Stopped point cloud publishing at " + stop_service;
      
      res.success = true;
      res.message = status_msg;

      ROS_INFO("%s", status_msg.c_str());
    }
  catch (const std::exception& e)
    {
      res.success = false;
      res.message = e.what();

      auto status_msg = "Failed to stop point cloud publishing, " +
                        stop_service + ": " + e.what();

      ROS_INFO("%s", status_msg.c_str());
      return false;
    }

  return true;
}

// ============================================================================
// ============================================================================
// ROS node initialization

void launch_node(int argc, char* argv[], const DeviceInfo& dinfo)
{
  ROS_INFO("Launching node: %s", dinfo.getNodeName().c_str());

  ros::NodeHandle n;

  // Start service server
  std::string start_service = dinfo.getStartServiceName();
  ROS_INFO("Starting service: %s", start_service.c_str());
  ros::ServiceServer service_start =
    n.advertiseService(start_service, start_service_callback);

  // Stop service server
  std::string stop_service = dinfo.getStopServiceName();
  ROS_INFO("Starting service: %s", stop_service.c_str());
  ros::ServiceServer service_stop =
    n.advertiseService(stop_service, stop_service_callback);

  publisher = std::make_shared<MkEPointCloudPublisher>(n, dinfo);
  ros::spin();
}

// ============================================================================

int start_publishing(int argc, char* argv[], const DeviceInfo& dinfo)
{
  ros::init(argc, argv, dinfo.getNodeName(), ros::init_options::AnonymousName);

  ros::NodeHandle n;
  auto start_service = dinfo.getStartServiceName();

  auto client_start = n.serviceClient<std_srvs::Trigger>(start_service);
  std_srvs::Trigger srv;

  ROS_INFO("Calling service:, %s", start_service.c_str());

  if (client_start.call(srv))
    {
      ROS_INFO("Service called successfully: %s", start_service.c_str());
      return 0;
    }
  else
    {
      ROS_ERROR("Service call failed, %s: %s", start_service.c_str(),
                srv.response.message.c_str());
      return 1;
    }

  return 0;
}

// ============================================================================

int stop_publishing(int argc, char* argv[], const DeviceInfo& dinfo)
{
  ros::init(argc, argv, dinfo.getNodeName(), ros::init_options::AnonymousName);

  ros::NodeHandle n;
  std::string stop_service = dinfo.getStopServiceName();

  auto stop_client = n.serviceClient<std_srvs::Trigger>(stop_service);
  std_srvs::Trigger srv;

  ROS_INFO("Calling service: %s", stop_service.c_str());

  if (stop_client.call(srv))
    {
      ROS_INFO("Service called successfully: %s", stop_service.c_str());
      return 0;      
    }
  else
    {
      ROS_ERROR("Service call failed, %s: %s", stop_service.c_str(),
                srv.response.message.c_str());
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
      ros::init(argc, argv, "MKEROS1_NODE", ros::init_options::AnonymousName);
      // ros::NodeHandle nh;

      if (ros::param::has("device"))
        {
          std::string device_param;
          ros::param::get("device", device_param);

          if (DeviceDiscovery::isIpAddress(device_param))
            {
              device_info.setIpAddr(device_param);
            }
          else
            {
              device_info.setUnitId(device_param);
            }
          
          if(ros::param::has("alias"))
            {
              std::string alias_param;
              ros::param::get("alias", alias_param);
              device_info.setAlias(alias_param);
            }
          else
            {
              device_info.setAlias();
            }
          
          ros::init(argc, argv, device_info.getNodeName());
          launch_node(argc, argv, device_info);
          
          return 0;
        }

      CliParser cli;
      cli.parseFromCmdLine(argc, argv);

      if (cli.help)
        {
          std::cout << "This is mkeros1_node v" << MKEROS1_VERSION 
                    << ", (c) MagikEye 2020-2021";
          std::cout << std::endl;

          cli.getCommandLineHelp(std::cout);
          return 0;
        }

      if (cli.version)
        {
          std::cout << MKEROS1_VERSION << std::endl;
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

          ros::init(argc, argv, device_info.getNodeName());

          launch_node(argc, argv, device_info);
        }
      else if (cli.start)
        {
          if (device_info.getName().empty())
          {
              throw std::runtime_error("Please set the '--device' or '--alias' option");
          }

          return start_publishing(argc, argv, device_info);
        }
      else if (cli.stop)
        {
          if (device_info.getName().empty())
          {
              throw std::runtime_error("Please set the '--device' or '--alias' option");
          }

          return stop_publishing(argc, argv, device_info);
        }
    }
  catch (std::exception& e)
    {
      std::cerr << "mkeros1_node fatal error: " << e.what() << std::endl;
      return 1;
    }

  return 0;
}