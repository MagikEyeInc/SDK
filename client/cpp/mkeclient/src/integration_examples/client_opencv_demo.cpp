
// MkE OpenCV demo
// Based on OpenCV tutorials here:
// https://docs.opencv.org/4.2.0/d7/df9/tutorial_table_of_content_viz.html

#include <iostream>
#include <thread>
#include <string>
#include <vector>

#include "mke/cli/client.h"
#include "mke/cli/bus.h"

#include <opencv2/viz.hpp>

using namespace mke::cli;
using namespace cv;
using namespace std;

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

void scale_openCV(uint32_t data3d_type, Mat& mat) {
  // Scaling factor to real world units (meters):
  double k = 0.001 / std::pow(2, data3d_type);
  mat = mat * k;
}

// ============================================================================

void get_single_frame_openCV(Client& client, Mat& mat)
{
  static Buffer buff;
  static mke::api::MkEReply_Frame reply;
  Frame frm = client.getFrame(mke::api::MKE_FRAME_TYPE_1, buff, reply);
  std::cout << "Have a frame with " << frm.size<mke::api::MkEFrameItem1>()
            << " points" << std::endl;

  frm.getOpenCV<mke::api::MkEFrameItem1>(mat);
  std::cout << "Have an OpenCV Mat with " << mat.total() << " points"
            << std::endl;

  scale_openCV(reply.data3d_type, mat);
}

// ============================================================================

void render_frame_openCV(viz::Viz3d& myWindow, Mat& cloud)
{
  viz::WCloud cloud_widget(cloud, viz::Color::green());
  cloud_widget.setRenderingProperty(viz::POINT_SIZE, 4.0);
  Affine3f cloud_pose = Affine3f().translate(Vec3f(0.0f, 0.0f, 0.0f));
  myWindow.showWidget("cloud", cloud_widget, cloud_pose);
  myWindow.spinOnce();
}

// ============================================================================

void run_demo(const char* host, const char* port = "8888")
{

  std::cout << "OpenCV version : " << CV_VERSION << std::endl;

  TcpBus tcp(host, std::stoi(port));
  Client client(&tcp);
  std::vector<std::vector<char>> buffers(10);
  int bi = 0;
  client.setPayloadBuffer(&buffers[bi++]);
  client.connect();
  try
    {
      switch_to_depth_sensor_state(client);

      std::mutex cloud_mutex;

#if (ENABLE_VISU)
      viz::Viz3d myWindow("OpenCV demo - press 'q' on keyboard to exit");

      Vec3f cam_pos(0.0f, 0.0f, -1.0f), cam_focal_point(0.0f, 0.0f, 1.0f),
        cam_y_dir(1.0f, 0.0f, 0.0f);
      Affine3f cam_pose =
        viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
      myWindow.setViewerPose(cam_pose);
      myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem(0.1f));
      myWindow.spinOnce();
#endif

      // Init Mat with one of supported types (CV_32FC3, CV_32SC3, CV_16SC3)
      // It will be resized to fit all incoming points
      Mat cloud(4000, 1, CV_32FC3);

      get_single_frame_openCV(client, cloud);

#if (ENABLE_VISU)
      render_frame_openCV(myWindow, cloud);
#endif

      std::cout << "Starting Frame Push." << std::endl;
      int idx = 1;
      client.startFramePush(
        mke::api::MKE_FRAME_TYPE_1,
        // Frame Callback
        [&idx, &client, &buffers, &bi, &cloud, &cloud_mutex
#if (ENABLE_VISU)
         ,
         &myWindow
#endif
      ](const mke::api::MkEReply_Frame& reply, const Frame& frm) {
          std::cout << "#" << idx++ << ": "
                    << frm.size<mke::api::MkEFrameItem1>() << " points"
                    << std::endl;

          cloud_mutex.lock();
          frm.getOpenCV<mke::api::MkEFrameItem1>(cloud);
          scale_openCV(reply.data3d_type, cloud);
          cloud_mutex.unlock();

#if (ENABLE_VISU)
        // Data is ready to display, but we are not displaying it here,
        // because viz::Viz3d can't be seamlessly updated from another thread
#endif

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
      int last_shown_idx = idx;
      while (!myWindow.wasStopped())
        {
          if (idx > last_shown_idx)
            {
              last_shown_idx = idx;
              cloud_mutex.lock();
              render_frame_openCV(myWindow, cloud);
              cloud_mutex.unlock();
            }
          else
            {
              myWindow.spinOnce();
            }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      myWindow.close();
#else
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
#endif

      std::cout << "Sending stop Frame Push." << std::endl;
      client.stopFramePush();

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
                    << "1. $./client_opencv_demo 192.168.0.100 8889"
                    << std::endl;
          std::cout << "2. $./client_opencv_demo 192.168.0.100" << std::endl;
        }
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}
