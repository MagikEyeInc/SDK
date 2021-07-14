// MkE Open3D demo

#include <iostream>
#include <thread>
#include <string>
#include <vector>

#include "mke/cli/client.h"
#include "mke/cli/bus.h"


#include <eigen3/Eigen/Dense>
#include "open3d/Open3D.h"
#include <cstdio>

using namespace open3d;
using namespace mke::cli;

bool stop_vis = false;

// ============================================================================

bool devs_callback(const open3d::visualization::Visualizer* viz)
{
  stop_vis = true;
  return true;
}

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

void get_single_frame_open3D(
  Client& client,
  std::shared_ptr<open3d::geometry::PointCloud>& o3d_ptr)
{

  static Buffer buff;
  static mke::api::MkEReply_Frame reply;
  Frame frm = client.getFrame(mke::api::MKE_FRAME_TYPE_1, buff, reply);
  std::cout << "Have a frame with " << frm.size<mke::api::MkEFrameItem1>()
            << " points" << std::endl;

  frm.getOpen3D<mke::api::MkEFrameItem1>(o3d_ptr);
  std::cout << "Have a Open3D PointCloud with " << o3d_ptr->points_.size()
            << " points" << std::endl;

  // This can be used to scale points to real world units (meters):
  // double k = 0.001 / std::pow(2, reply.data3d_type);
  // o3d_ptr->Scale(k, Eigen::Vector3d(0,0,0));
}

// ============================================================================

#if (ENABLE_VISU)
// Visualize Open3D Point Cloud:
void render_frame_open3D(visualization::VisualizerWithKeyCallback& viz,
                         std::shared_ptr<open3d::geometry::PointCloud>& o3d_ptr)
{

  o3d_ptr->GetAxisAlignedBoundingBox();
  viz.UpdateGeometry(o3d_ptr);
  viz.PollEvents();
  viz.UpdateRender();
}
#endif

// ============================================================================

void run_demo(const char* host, const char* port = "8888")
{

  std::cout << "Open3D version : " << OPEN3D_VERSION << std::endl;

  TcpBus tcp(host, std::stoi(port));
  Client client(&tcp);
  std::vector<std::vector<char>> buffers(10);
  int bi = 0;
  client.setPayloadBuffer(&buffers[bi++]);
  client.connect();
  try
    {
      switch_to_depth_sensor_state(client);

      std::shared_ptr<geometry::PointCloud> o3d_ptr(new geometry::PointCloud);

      get_single_frame_open3D(client, o3d_ptr);

#if (ENABLE_VISU)
      visualization::VisualizerWithKeyCallback viz;
      viz.CreateVisualizerWindow("Open3D demo - press 'q' on keyboard to exit",
                                 1920, 540);
      int key = 81; // 'q'
      viz.RegisterKeyCallback(key, devs_callback);
      auto bounding_box = o3d_ptr->GetAxisAlignedBoundingBox();
      viz.AddGeometry(o3d_ptr);
      render_frame_open3D(viz, o3d_ptr);
#endif

      o3d_ptr->points_.clear();

      std::cout << "Starting Frame Push." << std::endl;
      int idx = 1;
      client.startFramePush(
        mke::api::MKE_FRAME_TYPE_1,
        // Frame Callback
        [&idx, &client, &buffers, &bi, &o3d_ptr
#if (ENABLE_VISU)
         ,
         &viz
#endif
      ](const mke::api::MkEReply_Frame& params, const Frame& frm) {
          std::cout << "#" << idx++ << ": "
                    << frm.size<mke::api::MkEFrameItem1>() << " points"
                    << std::endl;

          frm.getOpen3D<mke::api::MkEFrameItem1>(o3d_ptr);

        // This can be used to scale points to real world units (meters):
        // double k = 0.001 / std::pow(2, reply.data3d_type);

#if (ENABLE_VISU)
          render_frame_open3D(viz, o3d_ptr);
#endif
          o3d_ptr->points_.clear();

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
      while (!stop_vis)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
#else
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
#endif

      std::cout << "Sending stop Frame Push." << std::endl;
      client.stopFramePush();

#if (ENABLE_VISU)
      viz.Close();
#endif

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
                    << "1. $./client_open3d_demo 192.168.0.100 8889"
                    << std::endl;
          std::cout << "2. $./client_open3d_demo 192.168.0.100" << std::endl;
        }
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}
