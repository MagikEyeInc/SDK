#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <iomanip>

#include "mke/cli/client.h"
#include "mke/cli/bus.h"

using namespace mke::cli;


// ============================================================================

uint64_t stamp_now()
{
  auto t0 = std::chrono::high_resolution_clock::now();
  return static_cast<uint64_t>(t0.time_since_epoch().count() / 1000000ULL);
}

// ============================================================================

void print_info(Client& client)
{
  mke::api::MkEReply_DeviceInfo dev_info = client.getDeviceInfo();
  std::cout << "Device ID: " << dev_info.device_id
            << ", Unit ID:" << dev_info.unit_id << std::endl;

  mke::api::MkEReply_FirmwareInfo fw_info = client.getFirmwareInfo();
  std::cout << "FW: " << int(fw_info.fw_ver_major) << "."
            << int(fw_info.fw_ver_minor) << "." << int(fw_info.fw_ver_patch)
            << std::endl;
  std::cout << "RT: " << int(fw_info.rt_ver_major) << "."
            << int(fw_info.rt_ver_minor) << "." << int(fw_info.rt_ver_patch)
            << std::endl;
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

void latency_app(const char* host, const char* port="8888", 
                 const char* samples="100", const char* out_file=nullptr)
{
  int num_samples=std::stoi(samples);
  mke::cli::TcpBus tcp(host, std::stoi(port));
  Client client(&tcp);

  Buffer buff;
  client.setPayloadBuffer(&buff);
  client.connect();

  try
    {
      print_info(client);

      switch_to_depth_sensor_state(client);

      std::vector<uint64_t> stamps(num_samples * 2);
      std::mutex m;
      std::condition_variable cv;
      float avg = 0;

      std::cout << "Starting Frame Push." << std::endl;
      int idx = 0;
      client.startFramePush(
        mke::api::MKE_FRAME_TYPE_1,
        // Frame Callback
        [&idx, &client, &stamps, num_samples, &m, &cv,
         &avg](const mke::api::MkEReply_Frame& params, const Frame& frm) {
          std::unique_lock<std::mutex> lock(m);
          if (idx >= num_samples)
            {
              if (idx == num_samples)
                cv.notify_one();
              return;
            }

          stamps[2 * idx] = stamp_now();
          stamps[2 * idx + 1] = params.timer;
          uint64_t latency = (stamps[2 * idx] - stamps[2 * idx + 1]);
          avg += latency;
          std::cout << "\r" << idx << "/" << num_samples << ": " << latency
                    << "ms, avg: " << avg / (1 + idx) << "ms"
                    << std::string(10, ' ');
          idx++;
        },
        // Status Callback
        [&cv](mke::api::MkEReplyStatus st) {
          if (st == mke::api::MKE_REPLY_DATA_STOPPED)
            cv.notify_one();
        },
        // Err Callback
        [](const mke::Error& e) {
          std::cout << "An error occured: " << e.what() << std::endl;
        });

      {
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock);
      }

      std::cout << std::endl << "Sending stop Frame Push." << std::endl;
      client.stopFramePush();
      client.setState(mke::api::MKE_STATE_IDLE);

      if (out_file)
        {
          std::ofstream ofs(out_file, std::ofstream::out);
          ofs << "timestamp(by_client);frame.timer" << std::endl;
          for (int i = 0; i < num_samples; ++i)
            ofs << std::fixed << std::setprecision(3) << stamps[2 * i] << ";"
                << stamps[2 * i + 1] << std::endl;
        }
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
          latency_app(argv[1]);
        }
      else if (argc == 3)
        {
          latency_app(argv[1], argv[2]);
        }
      else if (argc == 4)
        {
          latency_app(argv[1], argv[2], argv[3]);
        }
      else if (argc == 5)
        {
          latency_app(argv[1], argv[2], argv[3], argv[4]);
        }
      else
        {
          std::cout << "Expected one required argument and ";
          std::cout << "up to 3 optional arguments: " << std::endl;
          std::cout << "1) hostname or IP-address " << std::endl;
          std::cout << "2) port number of the sensor [default=8888] ";
          std::cout << std::endl;
          std::cout << "3) number of frames [default=100]" << std::endl;
          std::cout << "4) output filename [default=''] " << std::endl;
          std::cout << std::endl;
          std::cout << "Examples:" << std::endl;
          std::cout << "1. $./demo_pushframes 192.168.0.100" << std::endl;
          std::cout << "2. $./demo_pushframes 192.168.0.100 ";
          std::cout << "8888 100 output_file.csv" << std::endl;
        }  
    }
  catch (std::exception& e)
    {
      std::cout << "ERROR: " << e.what() << std::endl;
    }
}
