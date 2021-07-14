#include <iostream>
#include <cstdio>

#include "mke/cli/client.h"
#include "mke/cli/bus.h"


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
      // Get Device info:
      mke::api::MkEReply_DeviceInfo dev_info = client.getDeviceInfo();
      std::cout << "Device ID: " << dev_info.device_id
                << ", Unit ID:" << dev_info.unit_id << std::endl;

      // Get Firmware info:
      mke::api::MkEReply_FirmwareInfo fw_info = client.getFirmwareInfo();
      std::cout << "FW: " << int(fw_info.fw_ver_major) << "."
                << int(fw_info.fw_ver_minor) << "." << int(fw_info.fw_ver_patch)
                << std::endl;
      std::cout << "RT: " << int(fw_info.rt_ver_major) << "."
                << int(fw_info.rt_ver_minor) << "." << int(fw_info.rt_ver_patch)
                << std::endl;

      // Get Device state:
      mke::api::MkEStateType state = client.getState();
      std::cout << "Current state is: no." << state << std::endl;

      // Set Device state:
      if (state != mke::api::MKE_STATE_DEPTH_SENSOR)
        {
          client.setState(mke::api::MKE_STATE_DEPTH_SENSOR);
        }
      state = client.getState();
      std::cout << "Current state is: no." << state << std::endl;

      client.setState(mke::api::MKE_STATE_IDLE);

      state = client.getState();
      std::cout << "Current state is: no." << state << std::endl;
    }
  catch (const mke::cli::IOError& e)
    {
      std::cout << "IOError: " << e.what() << std::endl;
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
                    << "1. $./demo_device_state 192.168.0.100 8888"
                    << std::endl;
          std::cout << "2. $./demo_device_state 192.168.0.100" << std::endl;
        }
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}