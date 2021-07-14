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

void get_single_frame(mke::cli::Client& client)
{
  static mke::cli::Buffer buff;
  static mke::api::MkEReply_Frame reply;

  // Get frame:
  mke::cli::Frame frm =
    client.getFrame(mke::api::MKE_FRAME_TYPE_1, buff, reply);

  // Print frame info:
  const mke::api::MkEFrameItem1* items;
  items = frm.getAs<mke::api::MkEFrameItem1>();
  std::cout << "Have a frame with " << frm.size<mke::api::MkEFrameItem1>()
            << " points. The first 10 of them: " << std::endl;

  // Scaling factor to real world units (meters):
  double k = 0.001 / std::pow(2, reply.data3d_type);

  // Print first 10 points:
  for (int i = 0; i < frm.size<mke::api::MkEFrameItem1>(); ++i, ++items)
    {
      std::cout << (k * items->x) << "," << (k * items->y) << ","
                << (k * items->z) << std::endl;

      if (i >= 9)
        break;
    }
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

      for (int i = 0; i < 10; i++)
        {
          std::cout << "Getting 1 frame." << std::endl;
          get_single_frame(client);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
  catch (mke::cli::ServerFatalError& e)
    {
      std::cout << "Server fatal error: " << e.what() << std::endl;
    }
  catch (mke::cli::BadReplyError& e)
    {
      std::cout << "Bad reply error: " << e.what() << std::endl;
    }
  catch (mke::cli::IOError& e)
    {
      std::cout << "IO error: " << e.what() << std::endl;
    }
  catch (mke::Error& e)
    {
      std::cout << "Error: " << e.what() << std::endl;
    }
  catch (std::exception& e)
    {
      std::cout << "Other exception: " << e.what() << std::endl;
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
                    << "1. $./demo_getframe_sync 192.168.0.100 8888"
                    << std::endl;
          std::cout << "2. $./demo_getframe_sync 192.168.0.100" << std::endl;
        }
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}
