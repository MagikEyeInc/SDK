/*
 * publisher.h
 *
 * Copyright (c) 2020-2021, Magik-Eye Inc.
 * author: Jigar Patel, jigar@magik-eye.com
 */

#ifndef _PUBLISHER_
#define _PUBLISHER_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <mutex>
#include <thread>

#include "mke/cli/client.h"

#include "fpreal.h"
#include "device_info.h"

// ============================================================================
// MkEPointCloudPublisher

class MkEPointCloudPublisher : public rclcpp::Node
{
 private:
  DeviceInfo device_info_;
  std::atomic<bool> publishing_flag_;

  // TODO : Converted this to shared pointer - Check for Memory Leaks
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::unique_ptr<mke::cli::TcpBus> tcp_bus_;
  std::unique_ptr<mke::cli::Client> client_;

  std::mutex pmutex_;

  std::thread thread_;

  // ==========================================================================

  template <typename T>
  T convert(int16_t val, uint32_t type)
  {
    if (type == 0)
      return T(val);
    else if (type == 1)
      return T(mke::dd::types::fhreal1r(val, true));
    else if (type == 2)
      return T(mke::dd::types::fhreal2r(val, true));
    else if (type == 3)
      return T(mke::dd::types::fhreal3r(val, true));
    else if (type == 4)
      return T(mke::dd::types::fhreal4r(val, true));
  }

  // ==========================================================================

  void publish()
  {
    try
      {
        mke::cli::Buffer buff;
        publishing_flag_ = true;

        while (publishing_flag_)
          {
            mke::api::MkEReply_Frame params;

            const auto frame =
              client_->getFrame(mke::api::MKE_FRAME_TYPE_1, buff, params);

            sensor_msgs::msg::PointCloud2 cloud;
            sensor_msgs::PointCloud2Modifier modifier(cloud);

            const uint16_t num = frame.num_data;
            const uint32_t data3d_type = params.data3d_type;
            auto items = frame.getAs<mke::api::MkEFrameItem1>();
            const auto num_items = frame.size<mke::api::MkEFrameItem1>();

            // Resize modifier
            modifier.resize(num_items);

            // Add PointFields
            modifier.setPointCloud2Fields(
              4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
              sensor_msgs::msg::PointField::FLOAT32, "z", 1,
              sensor_msgs::msg::PointField::FLOAT32, "uid", 1,
              sensor_msgs::msg::PointField::FLOAT32);

            // Create Iters to add Data
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_uid(cloud, "uid");

            // Add Data
            for (int i = 0; i < (int)num_items; i++)
              {
                // Create Iters to add Data
                *iter_x = convert<float>(items->x, data3d_type);
                *iter_y = convert<float>(items->y, data3d_type);
                *iter_z = convert<float>(items->z, data3d_type);
                *iter_uid = convert<float>(items->uid, data3d_type);

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_uid;

                items++;
              }

            // Initialize other parameters
            cloud.header.frame_id = "Map";
            cloud.header.stamp = this->now();
            cloud.height = 1;
            cloud.width = num_items;
            cloud.is_bigendian = false;
            cloud.point_step = 16;
            cloud.row_step = cloud.point_step * num;
            cloud.is_dense = false;

            pub_->publish(cloud);
          }
      }
    catch (std::exception& e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                  "Error while publishing (%s:%s): %s",
                  device_info_.getUnitIdCStr(), device_info_.getIpAddrCStr(),
                  e.what());

        std::thread stop_publishing([this] { stopPublishing(); });
        stop_publishing.detach();
      }
  }

  // ==========================================================================

  void closeConnection()
  {
    client_.reset();
    tcp_bus_.reset();
    pub_.reset();
  }

  // ==========================================================================
  // ==========================================================================

 public:
  MkEPointCloudPublisher(const std::string& node_name,const DeviceInfo& device_info)
  : Node(node_name),device_info_{device_info}, publishing_flag_{false}
  {
  }

  // ==========================================================================

  ~MkEPointCloudPublisher()
  {
    stopPublishing();
  }

  // ==========================================================================

  void startPublishing()
  {
    std::lock_guard<std::mutex> lck(pmutex_);

    if (publishing_flag_)
      return;

    try
      {
        tcp_bus_.reset(
          new mke::cli::TcpBus(device_info_.getIpAddrCStr(), 8888));
        client_.reset(new mke::cli::Client(tcp_bus_.get()));

        client_->connect();

        // Go to MKE_STATE_DEPTH_SENSOR
        auto state = client_->getState();

        if (state == mke::api::MKE_STATE_IDLE)
          {
            client_->setState(mke::api::MKE_STATE_DEPTH_SENSOR);
          }
        else if (state != mke::api::MKE_STATE_DEPTH_SENSOR)
          {
            client_->setState(mke::api::MKE_STATE_IDLE);
            client_->setState(mke::api::MKE_STATE_DEPTH_SENSOR);
          }

        // Create topic publisher
        std::string topic_name = device_info_.getTopicName();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(device_info_.getTopicName(), 1);
      }
    catch (mke::cli::IOError &e)
      {
        const auto& dinfo = getDeviceInfo();
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                  "Connection broken (%s:%s)", dinfo.getUnitIdCStr(),
                  dinfo.getIpAddrCStr());
        exit(1);
      }
    catch (mke::cli::ServerFatalError &e)
      {
        const auto& dinfo = getDeviceInfo();
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                  "Connection broken (%s:%s)", dinfo.getUnitIdCStr(),
                  dinfo.getIpAddrCStr());
        exit(1);
      }
    catch (mke::Error e)
      {
        closeConnection();
        exit(1);
      }
    catch (...)
      {
        exit(1);
      }

    thread_ = std::thread([this] { publish(); });
  }

  // ==========================================================================

  void stopPublishing()
  {
    std::lock_guard<std::mutex> lck(pmutex_);

    if (!publishing_flag_)
      return;

    publishing_flag_ = false;
    thread_.join();

    if (!client_)
      {
        closeConnection();
        return;
      }

    try
      {
        auto state = client_->getState();

        if (state != mke::api::MKE_STATE_IDLE)
          {
            client_->setState(mke::api::MKE_STATE_IDLE);
          }
      }
    catch (const std::exception& e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                  "Sensor connection problem: %s", e.what());
      }

    closeConnection();
  }

  // ==========================================================================

  const DeviceInfo& getDeviceInfo() const
  {
    return device_info_;
  }

  // ==========================================================================

  void setDeviceInfo(const DeviceInfo& device_info)
  {
    device_info_ = device_info;
  }

};

#endif // _PUBLISHER_