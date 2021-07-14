#include <iostream>
#include <thread>
#include <cmath>
#include <cstdio>

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

void run_demo(const char* host, const char* port = "8888")
{
  mke::cli::TcpBus tcp(host, std::stoi(port));
  //  SerialBus serial("/dev/ttyUSB0", 115200);
  mke::cli::Client client(&tcp);
  std::vector<std::vector<char>> buffers(10);
  int bi = 0;
  client.setPayloadBuffer(&buffers[bi++]);
  client.connect();

  try
    {
      switch_to_depth_sensor_state(client);

      std::cout << "Starting Frame Push." << std::endl;
      int idx = 1;
      client.startFramePush(
        mke::api::MKE_FRAME_TYPE_1,
        // Frame Callback
        [&idx, &client, &buffers, &bi](const mke::api::MkEReply_Frame& reply,
                                       const mke::cli::Frame& frm) {
          std::cout << "#" << idx++ << ": "
                    << frm.size<mke::api::MkEFrameItem1>()
                    << " points. The first 10 of them: " << std::endl;

          // Scaling factor to real world units (meters):
          double k = 0.001 / std::pow(2, reply.data3d_type);

          // Load points:
          const mke::api::MkEFrameItem1* items =
            frm.getAs<mke::api::MkEFrameItem1>();

          // Print first 10 points:
          for (int i = 0; i < reply.num_data; ++i, items++)
            {
              std::cout << (k * items->x) << "," << (k * items->y) << ","
                        << (k * items->z) << std::endl;
              if (i >= 9)
                break;
            }

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
          std::cout << "Async error occurred: " << e.what() << std::endl;
        });

      std::this_thread::sleep_for(std::chrono::milliseconds(3000));

      std::cout << "Sending stop Frame Push." << std::endl;
      client.stopFramePush();

      client.setState(mke::api::MKE_STATE_IDLE);
    }
  catch (const mke::cli::IOError& e)
    {
      std::cout << "IOError: " << e.what() << std::endl;
    }
  catch (const std::exception& e)
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
                    << "1. $./demo_pushframes 192.168.0.100 8888" << std::endl;
          std::cout << "2. $./demo_pushframes 192.168.0.100" << std::endl;
        }
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}