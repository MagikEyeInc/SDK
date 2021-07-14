/*
 * Client - wraps communication protocol with the MagikEye sensor
 *               independently to a bus type
 * 
 * Copyright (c) 2017-2021, Magik-Eye Inc.
 * author: Ondra Fisar
 *
 */

/* -------------------------------------------------------------------------- */

#pragma once

/* -------------------------------------------------------------------------- */

#include <future>

/* -------------------------------------------------------------------------- */

#include <vector>
#include <queue>
#include <cstdint>
#include <stdexcept>
#include <cassert>
#include <functional>

/* -------------------------------------------------------------------------- */

#include "error.h"
#include "bus.h"

/* -------------------------------------------------------------------------- */
#if (USE_PCL)
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>
#endif

#if (USE_OPEN3D)
#include "open3d/Open3D.h"
#endif

#if (USE_OPENCV)
#include <opencv2/core.hpp>
#endif
/* -------------------------------------------------------------------------- */

#define TIMEOUT_INFINITE 0xffffffff

/* -------------------------------------------------------------------------- */


namespace mke {
namespace cli {

namespace priv {
  class RawClient;
}

/* -------------------------------------------------------------------------- */

/**
 * @brief Frame structure for holding frame's raw data bytes. This structure
 * doesn't hold type of data, correct FrameItemType should be used by user for
 * data retrieval.
 *
 */
struct Frame {
  
  const char * data;
  size_t num_data;

  Frame(const char * data = nullptr, size_t num = 0) :
    data(data), num_data(num)
  {};
  
  template <typename FrameItemType>
  inline const FrameItemType * getAs() const
  {
    return reinterpret_cast<const FrameItemType *>(data);
  }

#if (USE_PCL)
  template <typename FrameItemType>
  void getPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr& data_PCL) const {
    size_t pcSize = size<FrameItemType>();
    const FrameItemType* frame = reinterpret_cast<const FrameItemType*>(data);
    data_PCL->points.reserve(pcSize);
    for (size_t i = 0; i < pcSize; i++) {
      #if PCL_VERSION_COMPARE(<, 1, 10, 0)
        data_PCL->push_back(pcl::PointXYZ(frame[i].x, frame[i].y, frame[i].z));
      #else
        data_PCL->emplace_back(frame[i].x, frame[i].y, frame[i].z);
      #endif
    }
  }

#endif

#if (USE_OPEN3D)
  template <typename FrameItemType>
  void getOpen3D(
    std::shared_ptr<open3d::geometry::PointCloud>& data_O3D) const {
    size_t pcSize = size<FrameItemType>();
    const FrameItemType* frame = reinterpret_cast<const FrameItemType*>(data);
    data_O3D->points_.reserve(pcSize);
    for (size_t i = 0; i < pcSize; i++) {
      data_O3D->points_.emplace_back(frame[i].x, frame[i].y, frame[i].z);
    }
  }
#endif

#if (USE_OPENCV)
  /**
   * @brief Fill frame data to OpenCV Mat
   *
   * @param mat Output parameter that will be filled with data.
   * Type of Mat has to be one of these: CV_32FC3, CV_32SC3, CV_16SC3
   */
  template <typename FrameItemType>
  void getOpenCV(cv::Mat& mat) const {
    size_t pcSize = size<FrameItemType>();
    const FrameItemType* frame = reinterpret_cast<const FrameItemType*>(data);
    mat.resize(pcSize);
    if (mat.type()==CV_32FC3) { // 3x float
      cv::Point3f* data = mat.ptr<cv::Point3f>();  
      for(size_t i = 0; i < pcSize; i++) {
        data[i] = cv::Point3f(frame[i].x,frame[i].y,frame[i].z);
      }
    }
    else if (mat.type()==CV_32SC3) { // 3x signed 32bit integer
      cv::Point3i* data = mat.ptr<cv::Point3i>();  
      for(size_t i = 0; i < pcSize; i++) {
        data[i] = cv::Point3i(frame[i].x,frame[i].y,frame[i].z);
      }
    } 
    else if (mat.type()==CV_16SC3) { // 3x signed 16bit integer
      int16_t* data = mat.ptr<int16_t>();  
      for(size_t i = 0; i < pcSize; i++) {
        size_t dest=i*3;
        data[dest] = frame[i].x;
        data[dest+1] = frame[i].y;
        data[dest+2] = frame[i].z;
      }
    }
    else {
      throw mke::Error("Unsupported cv::Mat type");
    } 
  }
#endif

  template <typename FrameItemType>
  inline size_t size() const
  {
    if (num_data<=checksum_length_bytes) return 0; // num_data is in bytes
    return (num_data-checksum_length_bytes) / sizeof(FrameItemType);
  }
  
  bool isValid() const;

  private: 
    static const size_t checksum_length_bytes=sizeof(uint32_t);
};

/* -------------------------------------------------------------------------- */

typedef std::vector<char>   Buffer;

/* -------------------------------------------------------------------------- */

// acknowledge callback

typedef std::function<void()>                                   AckCallback;
typedef std::function<void(const mke::Error &)>                 ErrorCallback;

// parameters callback

typedef std::function<void(const api::MkEReplyStatus)>          StatusCallback;
typedef std::function<void(api::MkEStateType)>                  StateCallback;
typedef std::function<void(const api::MkEReply_FirmwareInfo&)>  FirmwareInfoCallback;
typedef std::function<void(const api::MkEReply_DeviceInfo&)>    DeviceInfoCallback;
typedef std::function<void(const char *)>                       PolicyCallback;
typedef std::function<void(const std::vector<std::string> &)>   ListPoliciesCallback;
typedef std::function<void(const api::MkEReply_Frame&, const Frame &)> 
                                                                FrameCallback;

/* -------------------------------------------------------------------------- */

// Client class
/**
 * @brief Client class provides simple interface for interaction with Device via
 * MkE API. After successful connection, you can send requests to Device by
 * using methods with either synchronous or asynchronous interface.
 *
 */
class Client

{ 
public:
  typedef uint32_t Handle;

  /**
   * @brief Contructor of the client
   * 
   * @param bus communication bus to be used
   */
  Client(BaseBus * bus);
  
  /**
   * @brief Destructor of the client
   */
  virtual ~Client();
  
  /**
   * @brief Get length (in bytes) of frame by its type 
   */
  virtual size_t frameLen(mke::api::MkEFrameType type);

  /**
   * @brief Connect to the device
   */
  void connect();

  /**
   * @brief Is the device connected?
   */
  bool is_connected() const;
  
  /**
   * @brief Disconnect from the device
   */
  void disconnect();
  
  /**
   * @brief Cancel request
   */
  void cancelRequest(Handle handle);
    
  /**
   * @brief Get current state of the device - async call
   */
  Handle getState(StateCallback callback, ErrorCallback errorCallback);
  
  /**
   * @brief Get current state of the device - sync call
   * 
   * @return current state of the device
   */
  api::MkEStateType getState(unsigned timeout_ms = TIMEOUT_INFINITE);
    
  /**
   * @brief Set payload buffer should be used
   * 
   * @param buffer payload buffer
   */
  void setPayloadBuffer(Buffer * buffer);
  
  /**
   * @brief Set state of the device - async call
   * 
   * @param state state should be set
   */
  Handle setState(api::MkEStateType state, AckCallback ackCallback, 
                  ErrorCallback errorCallback);
  
  /**
   * @brief Set state of the device - sync call
   * 
   * @param state state should be set
   */
  void setState(api::MkEStateType state, unsigned timeout_ms = TIMEOUT_INFINITE);
  
  /**
   * @brief Get firmware info of the device - async call
   */
  Handle getFirmwareInfo(FirmwareInfoCallback callback, ErrorCallback errorCallback);
  
  /**
   * @brief Get firmware info of the device - sync call
   * 
   * @return firmware info
   */
  mke::api::MkEReply_FirmwareInfo getFirmwareInfo(unsigned timeout_ms = TIMEOUT_INFINITE);
  
  /**
   * @brief Get device info of the device - async call
   */
  Handle getDeviceInfo(DeviceInfoCallback callback, ErrorCallback errorCallback);
  
  /**
   * @brief Get device info of the device - sync call
   * 
   * @return device info
   */
  mke::api::MkEReply_DeviceInfo getDeviceInfo(unsigned timeout_ms = TIMEOUT_INFINITE);
  
  /**
   * @brief Terminate device - Async call
   * 
   * @param term_type type of termination
   */
  Handle terminate(api::MkETerminateMethodType term_type, AckCallback ackCallback,
                   ErrorCallback errorCallback);
  
  /**
   * @brief Terminate device - sync call
   * 
   * @param term_type type of termination
   */
  void terminate(mke::api::MkETerminateMethodType term_type, unsigned timeout_ms = TIMEOUT_INFINITE);
  
  /**
   * @brief Get current frame from the device (depth-sensor) - async call
   * 
   * @param frame_type type of items given in frame
   */
  Handle getFrame(api::MkEFrameType frame_type, FrameCallback callback,
                  ErrorCallback errorCallback);
  
  /**
   * @brief Get current frame from the device (depth-sensor) - sync call
   * 
   * @param frame_type type of items given in frame
   * @param buff reference to array which should be filled with items
   * @return Frame convenient Frame structure pointing to provided Buffer
   */
  Frame getFrame(api::MkEFrameType frame_type, Buffer & buff, 
                 mke::api::MkEReply_Frame & params, unsigned timeout_ms = TIMEOUT_INFINITE);
  
  /**
   * @brief Start pushing frames - async call
   * 
   * @param frame_type type of items given in frame
   */
  Handle startFramePush(api::MkEFrameType frame_type, FrameCallback callback, 
                        StatusCallback transferCallback, ErrorCallback errorCallback);
  
  /**
   * @brief Stop pushing frames - async call
   */
  Handle stopFramePush(AckCallback ackCallback, ErrorCallback errorCallback);  
  
  /**
   * @brief Stop pushing frames - sync call
   */
  void stopFramePush(unsigned timeout_ms = TIMEOUT_INFINITE);

  /**
   * @brief Get policy - async call
   */
  Handle getPolicy(PolicyCallback callback, ErrorCallback err);

  /**
   * @brief Set policy - async call
   * 
   * @param policy policy to be set
   */
  Handle setPolicy(const char * policy, AckCallback stats, ErrorCallback err);

  /**
   * @brief List policies - async call
   */
  Handle listPolicies(ListPoliciesCallback callback, ErrorCallback err);

protected:
  priv::RawClient * raw_;
};

/* -------------------------------------------------------------------------- */

} // end of mke::cli namespace
} // end of mke namespace 

/* -------------------------------------------------------------------------- */

