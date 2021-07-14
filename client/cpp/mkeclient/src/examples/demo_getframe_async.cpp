#include <iostream>
#include <thread>
#include <cstdio>
#include <cmath>

#include "mke/cli/client.h"
#include "mke/cli/bus.h"

// ============================================================================

void switch_to_depth_sensor_state(mke::cli::Client& client)
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

void get_single_frame_async(mke::cli::Client& client)
{

  client.getFrame(
    mke::api::MKE_FRAME_TYPE_1,
    // Frame callback:
    [](const mke::api::MkEReply_Frame& reply, const mke::cli::Frame& frm) {
      // Copy incoming frame:
      mke::cli::Buffer frame_buffer;
      frame_buffer.reserve(frm.num_data); // resize
      frame_buffer.assign(frm.data, frm.data + frm.num_data);
      mke::cli::Frame frame =
        mke::cli::Frame(frame_buffer.data(), frame_buffer.size());

      // Print frame info:
      const mke::api::MkEFrameItem1* items;
      items = frame.getAs<mke::api::MkEFrameItem1>();
      std::cout << "Have a frame with " << frame.size<mke::api::MkEFrameItem1>()
                << " points. The first 10 of them: " << std::endl;

      // Scaling factor to real world units (meters):
      double k = 0.001 / std::pow(2, reply.data3d_type);

      // Print first 10 points:
      for (int i = 0; i < frame.size<mke::api::MkEFrameItem1>(); ++i, ++items)
        {
          std::cout << (k * items->x) << "," << (k * items->y) << ","
                    << (k * items->z) << std::endl;
          if (i >= 9)
            break;
        }
    },
    // Error callback:
    [&](const mke::Error& e) {
      std::cout << "Error callback: " << e.what() << std::endl;
    });
}

// ============================================================================

void run_demo(const char* host, const char* port = "8888")
{
  mke::cli::TcpBus tcp(host, std::stoi(port));
  //  SerialBus serial("/dev/ttyUSB0", 115200);
  mke::cli::Client client(&tcp);
  std::vector<std::vector<char>> buffers(10);
  int bi = 0;
  client.setPayloadBuffer(&buffers[bi++]);
  client.connect();

  switch_to_depth_sensor_state(client);

  for (int i = 0; i < 10; i++)
    {
      std::cout << "Getting 1 frame." << std::endl;
      get_single_frame_async(client);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

  try
    {
      client.setState(mke::api::MKE_STATE_IDLE);
    }
  catch (mke::cli::BadReplyError& e)
    {
      // This can happened when state is already set
      std::cout << "Bad reply error: " << e.what() << std::endl;
    }
  catch (mke::Error& e)
    {
      std::cout << "Error: " << e.what() << std::endl;
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
                    << "1. $./demo_getframe_async 192.168.0.100 8888"
                    << std::endl;
          std::cout << "2. $./demo_getframe_async 192.168.0.100" << std::endl;
        }
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}