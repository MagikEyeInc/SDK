// MkE PCL demo

#include <iostream>
#include <thread>
#include<memory>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

#include "mke/cli/client.h"
#include "mke/cli/bus.h"

#include <cstdio>
#include <vector>

using namespace mke::cli;

using pcl::PointCloud;

// ============================================================================

void switch_to_depth_sensor_state(Client& client)
{
  mke::api::MkEStateType state = client.getState();
  std::cout << "Current state is: no." << state << std::endl;

  if (state != mke::api::MKE_STATE_DEPTH_SENSOR)
    {
      client.setState(mke::api::MKE_STATE_DEPTH_SENSOR);
    }
  state = client.getState();
  std::cout << "Current state is: no." << state << std::endl;
}

// ============================================================================

void get_single_frame_pcl(Client& client,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_data)
{
  static Buffer buff;
  static mke::api::MkEReply_Frame reply;
  Frame frm = client.getFrame(mke::api::MKE_FRAME_TYPE_1, buff, reply);
  std::cout << "Have a frame with " << frm.size<mke::api::MkEFrameItem1>()
            << " points" << std::endl;

  frm.getPCL<mke::api::MkEFrameItem1>(pcl_data);
  std::cout << "Have a PCL PointCloud with " << pcl_data->points.size()
            << " points" << std::endl;

  // This can be used to scale points to real world units (meters):
  // double k = 0.001 / std::pow(2, reply.data3d_type);
}

// ============================================================================

#if (ENABLE_VISU)
// Visualize PCL Point Cloud:
void render_frame_pcl(std::shared_ptr<pcl::visualization::PCLVisualizer>& vis,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_data)
{

  if (!vis->updatePointCloud(pcl_data))
    {
      vis->addPointCloud(pcl_data);
      vis->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5);
    }
  vis->spinOnce();
}
#endif

// ============================================================================

void run_demo(const char* host, const char* port = "8888")
{

  std::cout << "PCL version : " << PCL_VERSION_PRETTY << std::endl;

  TcpBus tcp(host, std::stoi(port));
  Client client(&tcp);
  std::vector<std::vector<char>> buffers(10);
  int bi = 0;
  client.setPayloadBuffer(&buffers[bi++]);
  client.connect();

  try
    {
      switch_to_depth_sensor_state(client);

      std::cout << "Getting 1 frame." << std::endl;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data(
        new pcl::PointCloud<pcl::PointXYZ>);
      get_single_frame_pcl(client, pcl_data);

#if (ENABLE_VISU)
      std::shared_ptr<pcl::visualization::PCLVisualizer> vis(
        new pcl::visualization::PCLVisualizer("PCL demo - press 'q' on "
                                              "keyboard "
                                              "to exit"));
      render_frame_pcl(vis, pcl_data);
#endif

      pcl_data->clear();

      std::cout << "Starting Frame Push." << std::endl;
      int idx = 1;
      client.startFramePush(
        mke::api::MKE_FRAME_TYPE_1,
        // Frame Callback
        [&idx, &client, &buffers, &bi, &pcl_data
#if (ENABLE_VISU)
         ,
         &vis
#endif
      ](const mke::api::MkEReply_Frame& params, const Frame& frm) {
          std::cout << "#" << idx++ << ": "
                    << frm.size<mke::api::MkEFrameItem1>() << " points"
                    << std::endl;

          frm.getPCL<mke::api::MkEFrameItem1>(pcl_data);

        // This can be used to scale points to real world units (meters):
        // double k = 0.001 / std::pow(2, reply.data3d_type);

#if (ENABLE_VISU)
          render_frame_pcl(vis, pcl_data);
#endif
          pcl_data->clear();

          client.setPayloadBuffer(&buffers[bi]);
          bi = (bi + 1) % buffers.size();
        },
        // Status Callback
        [](mke::api::MkEReplyStatus st) {
          if (st == mke::api::MKE_REPLY_DATA_STOPPED)
            std::cout << "Data stopped correctly." << std::endl;
          else if (st == mke::api::MKE_REPLY_DATA_WILL_START)
            std::cout << "Data will start." << std::endl;
        },
        // Err Callback
        [](const mke::Error& e) {
          std::cout << "An error occurred: " << e.what() << std::endl;
        });

#if (ENABLE_VISU)
      while (!vis->wasStopped())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
#else
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
#endif

      std::cout << "Sending stop Frame Push." << std::endl;
      client.stopFramePush();

      pcl_data.reset();

      client.setState(mke::api::MKE_STATE_IDLE);
    }
  catch (std::exception& e)
    {
      std::cout << e.what() << std::endl;
      client.setState(mke::api::MKE_STATE_IDLE);
    }
}

// ============================================================================

int main(int argc, char* argv[])
{
  try
    {
      if (argc == 2)
        {
          run_demo(argv[1]);
        }
      else if (argc == 3)
        {
          run_demo(argv[1], argv[2]);
        }
      else
        {
          std::cout << "Expected one required argument and one optional argument"
                    << std::endl;
          std::cout << "Pass hostname or IP-address and optional port number of the sensor"
                    << std::endl;
          std::cout << "Examples:" << std::endl
                    << "1. $./client_pcl_demo 192.168.0.100 8889" << std::endl;
          std::cout << "2. $./client_pcl_demo 192.168.0.100" << std::endl;
        }
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}
