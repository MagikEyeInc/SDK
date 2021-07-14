/*
 * Magik Eye API 1.0
 *
 * Copyright (c) 2017-2021, Magik-Eye Inc
 * authors: Jan Heller, jan@magik-eye.com
 *          Ondra Fisar, fisar@magik-eye.com
 *
 */

#if (!defined(_MKEAPI_H_) && !defined(MKEAPI_C)) || \
    (!defined(_MKEAPI_C_H_) && defined(MKEAPI_C))

#if (!defined(_MKEAPI_H_) && !defined(MKEAPI_C))
#define _MKEAPI_H_
#endif

#if (!defined(_MKEAPI_C_H_) && defined(MKEAPI_C))
#define _MKEAPI_C_H_
#endif

#define _IN_MKEAPI_H_

#define MKEAPI_VERSION "1.0.0"

#ifdef MKERT_USE_RESERVED_API
#include "mkeapi_reserved.h"
#endif

/**
 * \defgroup mkeapi MkE API
 *
 */

#include <stdint.h>
#include <stddef.h>

#if defined(__cplusplus) && !defined(MKEAPI_C)
namespace mke {
namespace api {
#endif

/** \addtogroup mkeapi
 *  @{
 *
 */

// MkE State ==================================================================

#ifndef MKE_STATE_RESERVED
#define MKE_STATE_RESERVED
#endif


/**
 * @brief Enum type listing MkE API states, i.e., modes of operation of the sensor.
 *
 */
enum MkEStateType {
  MKE_STATE_UNDEF = 0, /**< For internal purposes only, does not name a valid state. */
  MKE_STATE_IDLE = 1,  /**< In this state, no 3D sensing is performed
                        and the sensor consumes only limited power resources. */
  MKE_STATE_DEPTH_SENSOR = 2, /**< In this state, the sensor computes the depth
      information and, upon request, provides this information to the client. */
  MKE_STATE_RESERVED  /**< For internal purposes only */
};

// MkE Request ================================================================

#ifndef MKE_REQUEST_RESERVED
#define MKE_REQUEST_RESERVED
#endif

#ifndef MKE_REQUEST_PARAMS_RESERVED
#define MKE_REQUEST_PARAMS_RESERVED
#endif

#ifndef MKE_TERMINATE_RESERVED
#define MKE_TERMINATE_RESERVED
#endif


/**
 * @brief Enum type corresponding to the legal values of `MkERequest::reqid`.
 *
 * Lists the names of commands a client of MkE API can use to ask the sensor to perform.
 */
enum MkERequestType {
  MKE_REQUEST_UNDEF = 0, 		/**< For internal purposes only, request of this type will not be accepted */
  MKE_REQUEST_TERMINATE = 10, 		/**< `MKE_REQUEST_TERMINATE` request is used to programatically shutdown or reboot the sensor. */
  MKE_REQUEST_GET_FIRMWARE_INFO = 11, 	/**< Use `MKE_REQUEST_GET_FIRMWARE_INFO` to query the sensors firmware information */
  MKE_REQUEST_GET_DEVICE_INFO = 12, 	/**< Use `MKE_REQUEST_GET_DEVICE_INFO` to query the sensors device information */
  MKE_REQUEST_GET_DEVICE_XML = 13, 	/**< Use `MKE_REQUEST_GET_DEVICE_XML` to query the sensors device information as an XML file */
  MKE_REQUEST_GET_STATE = 20, 		/**< Use `MKE_REQUEST_GET_STATE` to query the sensor's current state. */
  MKE_REQUEST_SET_STATE = 21, 		/**< Use `MKE_REQUEST_SET_STATE` to change the sensor's current state. */
  MKE_REQUEST_GET_POLICY = 22, 		/**< Use `MKE_REQUEST_GET_POLICY` to query the sensor's current policy. */
  MKE_REQUEST_SET_POLICY = 23, 		/**< Use `MKE_REQUEST_SET_POLICY` to change the sensor's current policy. */
  MKE_REQUEST_START_FRAME_PUSH = 24, 	/**< `MKE_REQUEST_STOP_START_PUSH` type to ellicit frame stream from the sensor. */
  MKE_REQUEST_STOP_FRAME_PUSH = 25, 	/**< Use `MKE_REQUEST_STOP_FRAME_PUSH` type to stop the frame stream ellicited by `MKE_REQUEST_START_FRAME_PUSH` request. */
  MKE_REQUEST_GET_FRAME = 26, 		/**< Use MKE_REQUEST_GET_FRAME type to request a single frame from the sensor.*/
  MKE_REQUEST_LIST_POLICIES = 27, 	/**< Use `MKE_REQUEST_LIST_POLICIES` to list all available policies. */
  MKE_REQUEST_INTERNAL = 1000,      	/**< For internal purposes only, internal request for socket termination */

  MKE_REQUEST_DYNAMIC_OFFSET = 2000, 	/**< For internal purposes only, the offset of dynamic requests */
  MKE_REQUEST_UPLOAD_PACKAGE = 2001, 	/**< Use MKE_REQUEST_UPLOAD_PACKAGE type to upload package to device */
  MKE_REQUEST_DYNAMIC_LAST = 2999, 	/**< For internal purposes only, Last dynamic request */
  MKE_REQUEST_RESERVED 			/**< For internal purposes only */

};

/**
 * @brief Enum type listing possible runtime termination methods
 *
 */
enum MkETerminateMethodType {
  MKE_TERMINATE_UNDEF = 0, /**< For internal purposes only */
  MKE_TERMINATE_BY_REBOOT = 1, /**< Reboot the sensor */
  MKE_TERMINATE_BY_SHUTDOWN = 2, /**< Shutdown the sensor */
  MKE_TERMINATE_BY_EXIT_RESERVED = 4, /**< For internal purposes only */
  MKE_TERMINATE_RESERVED  /**< For internal purposes only */
};

/**
 * @brief Parameter structure of the `MKE_REQUEST_TERMINATE` request.
 *
 * It is used to shutdown or reboot the sensor programatically.
 */
struct MkERequest_Terminate {
  uint32_t  method;             /**< Termination method, see  `MkETerminateMethodType`. */
  uint8_t   undefined[4];       /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_Terminate) == 8, "Incorrect compilation size: MkERequest_Terminate");
static_assert(offsetof(MkERequest_Terminate, method) == 0, "Incorrect alignment (MkERequest_Terminate, method)");
static_assert(offsetof(MkERequest_Terminate, undefined) == 4, "Incorrect alignment (MkERequest_Terminate, undefined)");
#endif

/**
 * @brief Parameter structure of the `MKE_REQUEST_SET_STATE` request.
 *
 * It is used to set or change the sensor's current state.
 */
struct MkERequest_SetState {
  uint32_t  new_state;          /**< New state, see `MkEStateType`. */
  uint8_t   undefined[4];       /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_SetState) == 8, "Incorrect compilation size: MkERequest_SetState");
static_assert(offsetof(MkERequest_SetState, new_state) == 0, "Incorrect alignment (MkERequest_SetState, new_state)");
static_assert(offsetof(MkERequest_SetState, undefined) == 4, "Incorrect alignment (MkERequest_SetState, undefined)");
#endif

/**
 * @brief Parameter structure of the `MKE_REQUEST_GET_FRAME` and `MKE_REQUEST_START_FRAME_PUSH` requests.
 *
 * It is used to request a single frame from the sensor. The sensor canrequest frame items of two types.
 */
struct MkERequest_GetFrame {
  uint16_t  frame_type;         /**< Requested frame item type, See `MkEFrameType` */
  uint8_t   undefined[6];       /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_GetFrame) == 8, "Incorrect compilation size: MkERequest_GetFrame");
static_assert(offsetof(MkERequest_GetFrame, frame_type) == 0, "Incorrect alignment (MkERequest_GetFrame, frame_type)");
static_assert(offsetof(MkERequest_GetFrame, undefined) == 2, "Incorrect alignment (MkERequest_GetFrame, undefined)");
#endif

/**
 * @brief Parameter structure of the `MKE_REQUEST_SET_POLICY`.
 *
 * It is used to set the sensor's policy.
 */
struct MkERequest_SetPolicy {
  char      policy_name[8];     /**< Name of the policy to be set */
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_SetPolicy) == 8, "Incorrect compilation size: MkERequest_SetPolicy");
static_assert(offsetof(MkERequest_SetPolicy, policy_name) == 0, "Incorrect alignment (MkERequest_SetPolicy, policy_name)");
#endif

/**
 * @brief Represents the MkE API  Dynamic request.
 *
 * It is used to upload general variable-sized data to the sensor.
 * The format and interpretation of the uploaded data
 * is not part of the MkE API.
 * In practice, this request is used to upload firmware
 * binary package or other updates. The size of the uploaded data is limited to 64 MiB.
 */
struct MkERequest_UploadPackage_DynamicParams {
  uint32_t  crc32;		/**< CRC-32 checksum (ITU-T V.42) of the data payload */
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_UploadPackage_DynamicParams) == 4, "Incorrect compilation size: MkERequest_UploadPackage_DynamicParams");
#endif

union MkERequest_Dynamic_Params {
  char param_bytes[4];
  struct MkERequest_UploadPackage_DynamicParams param_upload_package;
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_Dynamic_Params) == 4, "Incorrect compilation size: MkERequest_Dynamic_Params");
static_assert(offsetof(MkERequest_Dynamic_Params, param_bytes) == 0, "Incorrect alignment (MkERequest_Dynamic_Params, param_bytes)");
static_assert(offsetof(MkERequest_Dynamic_Params, param_upload_package) == 0, "Incorrect alignment (MkERequest_Dynamic_Params, param_upload_package)");
#endif

struct MkERequest_Dynamic {
  uint32_t  payload_size;       /**< Size of payload after the request */
  union MkERequest_Dynamic_Params
            params;             /**< Rest of the parameters */
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_Dynamic) == 8, "Incorrect compilation size: MkERequest_Dynamic");
static_assert(offsetof(MkERequest_Dynamic, payload_size) == 0, "Incorrect alignment (MkERequest_Dynamic, payload_size)");
static_assert(offsetof(MkERequest_Dynamic, params) == 4, "Incorrect alignment (MkERequest_Dynamic, params)");
#endif

/**
 * @brief Represents the respective MkE API request parameters.
 *
 * This structure is always 8 bytes long.
 */
struct MkERequest_Params {
  union {
    uint8_t param_bytes[8];       /**< Allows for per-byte access to parameter data   */

    struct MkERequest_Terminate  terminate_params;   /**< See `MkERequest_Terminate` */
    struct MkERequest_GetFrame   getframe_params;    /**< See `MkERequest_GetFrame`*/
    struct MkERequest_SetState   setstate_params;    /**< See `MkERequest_SetState` */
    struct MkERequest_SetPolicy  setpolicy_params;   /**< See `MkERequest_SetPolicy` */
    struct MkERequest_Dynamic    dynamic_params;     /**< See `MkERequest_Dynamic` */
    MKE_REQUEST_PARAMS_RESERVED /**< For internal purposes only */
  };
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest_Params) == 8, "Incorrect compilation size: MkERequest_Params");
static_assert(offsetof(MkERequest_Params, param_bytes) == 0, "Incorrect alignment (MkERequest_Params, param_bytes)");
static_assert(offsetof(MkERequest_Params, terminate_params) == 0, "Incorrect alignment (MkERequest_Params, terminate_params)");
static_assert(offsetof(MkERequest_Params, getframe_params) == 0, "Incorrect alignment (MkERequest_Params, getframe_params)");
static_assert(offsetof(MkERequest_Params, setstate_params) == 0, "Incorrect alignment (MkERequest_Params, setstate_params)");
static_assert(offsetof(MkERequest_Params, setpolicy_params) == 0, "Incorrect alignment (MkERequest_Params, setpolicy_params)");
static_assert(offsetof(MkERequest_Params, dynamic_params) == 0, "Incorrect alignment (MkERequest_Params, params)");
#endif

/**
 * @brief Represents the MkE API request.
 *
 * MkE API request is always 24 bytes long, starting
 * with the string 'MKERQ100'. Parameters of the respective API request can be set via the
 * `MkERequest_Params` member.
 */
struct MkERequest {
  char magik[8];             /**< MkE API request packet identifier, must be set to : "MKERQ100". */
  char type[4];              /**< Request type as a zero padded decimal string,
                                  e.g., "0001" for request type 1.        */
  uint32_t reqid;            /**< ID number of a request. A respective
                                  response will have the same ID. */
  struct MkERequest_Params params; /**< Represents the respective MkE API request parameters. */
};

#ifdef __cplusplus
static_assert(sizeof(MkERequest) == 24, "Incorrect compilation size: MkERequest");
static_assert(offsetof(MkERequest, magik) == 0, "Incorrect alignment (MkERequest, magik)");
static_assert(offsetof(MkERequest, type) == 8, "Incorrect alignment (MkERequest, type)");
static_assert(offsetof(MkERequest, reqid) == 12, "Incorrect alignment (MkERequest, reqid)");
static_assert(offsetof(MkERequest, params) == 16, "Incorrect alignment (MkERequest, params)");
#endif

// MkE Reply ================================================================

#ifndef MKE_REPLY_PARAMS_RESERVED
#define MKE_REPLY_PARAMS_RESERVED
#endif


/**
 * @brief Enum type listing valid MkE reply statuses
 *
 */
enum MkEReplyStatus {
  MKE_REPLY_UNDEF = 0, /**< For internal purposes only */

  MKE_REPLY_DATA_WILL_START = 100, /**< Signalizes the successful initialization
                                      of the frame streaming process. */
  MKE_REPLY_DATA_WILL_CONTINUE = 101, /**< Signalizes that the frame stream will
                                   continue with at least one more data packet.*/
  MKE_REPLY_DATA_STOPPED = 102, /**<  Signalizes that the frame stream has been
               successfully stopped via `MKE_REQUEST_STOP_FRAME_PUSH` request.*/

  MKE_REPLY_OK = 200, /**< Signalizes that a request has been successfully handled.*/

  MKE_REPLY_CLIENT_ERROR = 400, /**< Signalizes a general client side error. */
  MKE_REPLY_CLIENT_MALFORMED_REQUEST = 401, /**< Signalizes a sensor's problem
                                                with parsing a request.*/
  MKE_REPLY_CLIENT_ILLEGAL_REQUEST_TYPE = 402, /**< Signalizes that the client issued
                                 a request type not available in the current state.*/
  MKE_REPLY_CLIENT_REQUEST_DOES_NOT_APPLY = 403, /**< Signalizes a situation
  where a client requested resources that were not available in the sensor's current state.*/
  MKE_REPLY_CLIENT_REQUEST_PAYLOAD_TOO_LONG = 404, /**< Signalizes that the payload of the
                                                      request was longer then allowed */

  MKE_REPLY_SERVER_ERROR = 500, /**< Signalizes a general sensor side error. */
  MKE_REPLY_SERVER_REQUEST_INTERRUPTED = 501, /**< Signalizes that a sensor's work
                                  on a reply has been externally interrupted.*/
  MKE_REPLY_SERVER_BUSY = 502, /**< The sensor will issue the `MKE_REPLY_SERVER_BUSY`
                                reply in situations where client requested an operation
                                that is already being processed by the sensor.*/
  MKE_REPLY_SERVER_INSUFFICIENT_RESOURCES = 503, /**< Signalizes fatal problem
                                  with memory resources on the sensor's side. */
  MKE_REPLY_SERVER_FATAL_ERROR = 504, /**< Signalizes fatal error
                                  that affects regular run of application, application is in FAILSAFE */
};


/**
 * @brief Parameter structure of the sensor's `Reply` to `MKE_REQUEST_GET_FRAME` and `MKE_REQUEST_START_FRAME_PUSH` requests.
 *
 * Whenever the `Reply`'s payload is to be interpreted as 3D data frame, its
 * parameters will have this structure.
 */
struct MkEReply_Frame {
  uint64_t timer;          /**< Time of the end of frame exposure in milliseconds elapsed from the boot.
                              Not particularly useful on its own, but can be used to measure
                              time elapsed between different frame exposures. */
  uint64_t seqn;           /**< Sequence number of a frame, starting from 0. This counter
                              is incremented every time a frame is successfully processed
                              by the sensor in the `MKE_STATE_DEPTH_SENSOR` state. */
  uint32_t data3d_type;    /**< Determines the units of the 3D coordinates of the frame items. Currently, there are five possible types. */
  uint16_t frame_type;     /**< Type of FrameData contained in the frame, see `MkEFrameType` */
  uint16_t num_data;       /**< Number of frame items contained in the frame. */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_Frame) == 24, "Incorrect compilation size: MkEReply_Frame");
static_assert(offsetof(MkEReply_Frame, timer) == 0, "Incorrect alignment (MkEReply_Frame, timer)");
static_assert(offsetof(MkEReply_Frame, seqn) == 8, "Incorrect alignment (MkEReply_Frame, seqn)");
static_assert(offsetof(MkEReply_Frame, data3d_type) == 16, "Incorrect alignment (MkEReply_Frame, data3d_type)");
static_assert(offsetof(MkEReply_Frame, frame_type) == 20, "Incorrect alignment (MkEReply_Frame, frame_type)");
static_assert(offsetof(MkEReply_Frame, num_data) == 22, "Incorrect alignment (MkEReply_Frame, num_data)");
#endif

/**
 * @brief Parameter structure of the sensor's `Reply` to `MKE_REQUEST_GET_STATE` request.
 *
 * This request is used to query the sensor's current state. There are two valid sensor states.
 */
struct MkEReply_State {
  uint32_t  state;         /**< Sensor's current state, see `MkEStateType` */
  char      undefined[20]; /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_State) == 24, "Incorrect compilation size: MkEReply_State");
static_assert(offsetof(MkEReply_State, state) == 0, "Incorrect alignment (MkEReply_State, state)");
static_assert(offsetof(MkEReply_State, undefined) == 4, "Incorrect alignment (MkEReply_State, undefined)");
#endif

/**
 * @brief Parameter structure of the sensor's `Reply` to `MKE_REQUEST_GET_FIRMWARE_INFO` request.
 *
 * This request is used to query the sensor’s firmware version and various other information.
 */
struct MkEReply_FirmwareInfo {
  int64_t   posix_time;    /**< POSIX time in the time of firmware compilation */
  uint32_t  git_commit;    /**< Short git hash of the latest firmware commit */
  uint8_t   rt_ver_major;  /**< Runtime version - major part*/
  uint8_t   rt_ver_minor;  /**< Runtime version - minor part */
  uint8_t   rt_ver_patch;  /**< Runtime version - patch part */
  uint8_t   fw_ver_major; /**< Firmware version - major part*/
  uint8_t   fw_ver_minor; /**< Firmware version - minor part */
  uint8_t   fw_ver_patch; /**< Firmware version - patch part */
  char      undefined[6];  /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_FirmwareInfo) == 24, "Incorrect compilation size: MkEMkEReply_FirmwareInfoReply_Frame");
static_assert(offsetof(MkEReply_FirmwareInfo, posix_time) == 0, "Incorrect alignment (MkEReply_FirmwareInfo, posix_time)");
static_assert(offsetof(MkEReply_FirmwareInfo, git_commit) == 8, "Incorrect alignment (MkEReply_FirmwareInfo, git_commit)");
static_assert(offsetof(MkEReply_FirmwareInfo, rt_ver_major) == 12, "Incorrect alignment (MkEReply_FirmwareInfo, rt_ver_major)");
static_assert(offsetof(MkEReply_FirmwareInfo, rt_ver_minor) == 13, "Incorrect alignment (MkEReply_FirmwareInfo, rt_ver_minor)");
static_assert(offsetof(MkEReply_FirmwareInfo, rt_ver_patch) == 14, "Incorrect alignment (MkEReply_FirmwareInfo, rt_ver_patch)");
static_assert(offsetof(MkEReply_FirmwareInfo, fw_ver_major) == 15, "Incorrect alignment (MkEReply_FirmwareInfo, fw_ver_major)");
static_assert(offsetof(MkEReply_FirmwareInfo, fw_ver_minor) == 16, "Incorrect alignment (MkEReply_FirmwareInfo, fw_ver_minor)");
static_assert(offsetof(MkEReply_FirmwareInfo, fw_ver_patch) == 17, "Incorrect alignment (MkEReply_FirmwareInfo, fw_ver_patch)");
static_assert(offsetof(MkEReply_FirmwareInfo, undefined) == 18, "Incorrect alignment (MkEReply_FirmwareInfo, undefined)");
#endif

/**
 * @brief Parameter structure of the sensor's `Reply` to `MKE_REQUEST_GET_DEVICE_INFO` request.
 *
 * This request is used to query the sensor's ID information.
 */
struct MkEReply_DeviceInfo {
  uint16_t  device_id;      /**< Identification of device. This value is shared with other devices of the same model. */
  char      unit_id[8];     /**< Unit ID or serial number of the device. */
  char      undefined[14];  /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_DeviceInfo) == 24, "Incorrect compilation size: MkEReply_DeviceInfo");
static_assert(offsetof(MkEReply_DeviceInfo, device_id) == 0, "Incorrect alignment (MkEReply_DeviceInfo, device_id)");
static_assert(offsetof(MkEReply_DeviceInfo, unit_id) == 2, "Incorrect alignment (MkEReply_DeviceInfo, unit_id)");
static_assert(offsetof(MkEReply_DeviceInfo, undefined) == 10, "Incorrect alignment (MkEReply_DeviceInfo, undefined)");
#endif

/**
 * @brief Parameter structure of the sensor's `Reply` to `MKE_REQUEST_GET_POLICY` request.
 *
 * The current policy can be queried via this request. The `MKE_REQUEST_GET_POLICY` request has no parameters. This means that the bytes of the request’s params field should be set to zero.
 */
struct MkEReply_GetPolicy {
  char      policy_name[8];  /**< Name of the current policy as a C-style string,i.e., zero terminated string. In the case the name is exactly 8 characters long, the terminating zero character is not added. */
  char      undefined[16];   /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_GetPolicy) == 24, "Incorrect compilation size: MkEReply_GetPolicy");
static_assert(offsetof(MkEReply_GetPolicy, policy_name) == 0, "Incorrect alignment (MkEReply_GetPolicy, policy_name)");
static_assert(offsetof(MkEReply_GetPolicy, undefined) == 8, "Incorrect alignment (MkEReply_GetPolicy, undefined)");
#endif

/**
 * @brief Parameter structure of the sensor's `Reply` to `MKE_REQUEST_LIST_POLICIES` request.
 *
 * This is used to list available policies.
 */
struct MkEReply_ListPolicies {
  uint32_t  num_policies;    /**< Number of the sensor specific policies. */
  char      undefined[20];   /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_ListPolicies) == 24, "Incorrect compilation size: MkEReply_ListPolicies");
static_assert(offsetof(MkEReply_ListPolicies, num_policies) == 0, "Incorrect alignment (MkEReply_ListPolicies, num_policies)");
static_assert(offsetof(MkEReply_ListPolicies, undefined) == 4, "Incorrect alignment (MkEReply_ListPolicies, undefined)");
#endif

/**
 * @brief Enum type listing fatal error types specifying the general error code in `MKE_REPLY_SERVER_FATAL_ERROR`.
 *
 */
enum MkEFatalErrorType {
  MKE_FATAL_UNDEF               = 0, /**< Unspecified fatal error */
  MKE_FATAL_BADCONFIG           = 1, /**< The device has corrupted configuration */
  MKE_FATAL_DETECTORINIT        = 2, /**< Unable to initialize the detector */
  MKE_FATAL_BADCAMERA           = 3, /**< The device has encountered a problem with the camera connection */
  MKE_FATAL_RUNTIME             = 4, /**< Unspecified fatal error during runtime */
  MKE_FATAL_SN_INCONSISTENCY    = 5, /**< Calibration is not compatible with current sensor */
};

/**
 * @brief Parameter structure of the sensor's `Reply` specifying the general error code `MKE_REPLY_SERVER_FATAL_ERROR`.
 *
 * It signals a fatal problem encountered during the sensor startup and runtime.
 * This problem may have been caused by hardware issues or by an unsuccessful firmware update.
 */
struct MkEReply_ServerFatal {
  uint32_t  err_code;      /**< Fatal error code, see `MkEFatalErrorType`. Supplementary error code specifying the problem as a valid value of MkEFatalErrorType. */
  char      undefined[20]; /**< Padding, should be set to zero */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_ServerFatal) == 24, "Incorrect compilation size: MkEReply_ServerFatal");
static_assert(offsetof(MkEReply_ServerFatal, err_code) == 0, "Incorrect alignment (MkEReply_ServerFatal, err_code)");
static_assert(offsetof(MkEReply_ServerFatal, undefined) == 4, "Incorrect alignment (MkEReply_ServerFatal, undefined)");
#endif

/**
 * @brief Represents the respective MkE API reply parameters.
 *
 * This structure is always 24 bytes long.
 */
struct MkEReply_params {
  union {
    uint8_t                        param_bytes[24]; /**< Allows for per-byte access to parameter data */
    struct MkEReply_Frame          frame_params;    /**< See `MkEFrameHeader` */
    struct MkEReply_State          state_params;    /**< See `MkEReply_State` */
    struct MkEReply_FirmwareInfo   fw_params;       /**< See `MkEReply_FirmwareInfo` */
    struct MkEReply_DeviceInfo     device_params;   /**< See `MkEReply_DeviceInfo` */
    struct MkEReply_GetPolicy      policy_params;   /**< See `MkEReply_GetPolicy` */
    struct MkEReply_ListPolicies   list_policies_params;   /**< See `MkEReply_ListPolicies` */
    struct MkEReply_ServerFatal    fatal_params;    /**< See `MkEReply_ServerFatal` */
    MKE_REPLY_PARAMS_RESERVED  /**< For internal purposes only */
  };
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply_params) == 24, "Incorrect compilation size: MkEReply_params");
static_assert(offsetof(MkEReply_params, param_bytes) == 0, "Incorrect alignment (MkEReply_params, param_bytes)");
static_assert(offsetof(MkEReply_params, frame_params) == 0, "Incorrect alignment (MkEReply_params, frame_params)");
static_assert(offsetof(MkEReply_params, fw_params) == 0, "Incorrect alignment (MkEReply_params, fw_params)");
static_assert(offsetof(MkEReply_params, device_params) == 0, "Incorrect alignment (MkEReply_params, device_params)");
static_assert(offsetof(MkEReply_params, policy_params) == 0, "Incorrect alignment (MkEReply_params, policy_params)");
static_assert(offsetof(MkEReply_params, list_policies_params) == 0, "Incorrect alignment (MkEReply_params, list_policies_params)");
static_assert(offsetof(MkEReply_params, fatal_params) == 0, "Incorrect alignment (MkEReply_params, fatal_params)");
#endif

/**
 * @brief Represents the reply to the MkE API request.
 *
 * The sensor reply data packet is always at least 48 bytes long.
 */
struct MkEReply {
  char magik[8];      /**< MkE API reply packet identifier. This must be set to: "MKERP100". */
  char type[4];       /**< Request type as a zero padded decimal string, of the original Mke API request that this reply replies to.
                                  i.e. "0001" for request type 1.    */
  char status[4];     /**< Reply status as a zero padded decimal string,
                           e.g. "0200" for "Server OK" (`MkEReplyStatus::MKE_REPLY_OK`)  */
  uint32_t reqid;    /**< ID number of a request this reply replies to. */
  uint32_t num_bytes;  /**< Size of the additional data directly following this reply. Indicates that an appropriate number of bytes will directly followthe first 48 bytes of reply.
              In the case there is no additional payload data, `num_bytes` must be set to 0.  */
  struct MkEReply_params params; /**< 24-bytes long data strucutre that represents the respective MkE API reply parameters. */
};

#ifdef __cplusplus
static_assert(sizeof(MkEReply) == 48, "Incorrect compilation size: MkEReply");
static_assert(offsetof(MkEReply, magik) == 0, "Incorrect alignment (MkEReply, magik)");
static_assert(offsetof(MkEReply, type) == 8, "Incorrect alignment (MkEReply, type)");
static_assert(offsetof(MkEReply, status) == 12, "Incorrect alignment (MkEReply, status)");
static_assert(offsetof(MkEReply, reqid) == 16, "Incorrect alignment (MkEReply, reqid)");
static_assert(offsetof(MkEReply, num_bytes) == 20, "Incorrect alignment (MkEReply, num_bytes)");
static_assert(offsetof(MkEReply, params) == 24, "Incorrect alignment (MkEReply, params)");
#endif

// MkE Frame Data ================================================================

#ifndef MKE_FRAME_TYPE_RESERVED
#define MKE_FRAME_TYPE_RESERVED
#endif

/**
The 3D data from the sensor is provided as a reply data payload, which in this context is called a frame. MkE frame data (or 3D data) consists a variable number of data entries (MkEFrameData1,
MkEFrameData2, MkEFrameData3, or MkEFrameData4) of the same type and a frame footer.
The data entries are in little endian format and the memory layout of
the MkE frame data is as follows:

`MkEFrameType1`
`MkEFrameType2`
...
`MkEFrameType*`
`MkEFrameFooter`
 */

/**
 * @brief MkE 3D data frame footer.
 *
 * The 3D Data frame consists of frame items followed by a frame footer.
 */
struct MkEFrameFooter {
  uint32_t crc32;      /**< CRC-32 checksum (ITU-T V.42) of the frame items bytes. */
};

#ifdef __cplusplus
static_assert(sizeof(MkEFrameFooter) == 4, "Incorrect compilation size: MkEFrameFooter");
static_assert(offsetof(MkEFrameFooter, crc32) == 0, "Incorrect alignment (MkEFrameFooter, crc32)");
#endif

/**
 * @brief Enum type listing valid 3D frame item types. The frame itself consists of a variable number of frame items--which conceptually correspond to detections, i.e., 3D points.
 *
 */
enum MkEFrameType {
  MKE_FRAME_TYPE_UNDEF = 0, /**< For internal purposes only */
  MKE_FRAME_TYPE_1 = 1, /**< `MKE_FRAME_TYPE_1` frame type */
  MKE_FRAME_TYPE_2 = 2, /**< `MKE_FRAME_TYPE_2` frame type */
  MKE_FRAME_TYPE_RESERVED /**< For internal purposes only */
};

/**
 * @brief Enum type listing valid 3D data types (units) that the sensor can use to encode the 3D coordinate values. The 3D data type is determined by the sensor and cannot be changed by an MkERequest. However, it is possible for the sensor to change the 3D data type during its operations.
 *
 */
enum MkEData3dType {
  MKE_DATA3D_MM = 0,     /**< 1 millimeter resolution */
  MKE_DATA3D_MM2 = 1,    /**< 1/2 millimeter resolution */
  MKE_DATA3D_MM4 = 2,    /**< 1/4 millimeter resolution */
  MKE_DATA3D_MM8 = 3,    /**< 1/8 millimeter resolution */
  MKE_DATA3D_MM16 = 4    /**< 1/16 millimeter resolution */
};

/**
 * @brief Inner structure of the frame item type `MKE_FRAME_TYPE_1`
 *
 */
struct MkEFrameItem1 {
  uint16_t uid; /**< Universal ID key of the detetction */
  int16_t  x;   /**< X coordinate of the measured 3D point. Unit is determined by data3d_type in the coordinate system connected with the sensor. */
  int16_t  y;   /**< Y coordinate of the measured 3D point. Unit is determined by data3d_type in the coordinate system connected with the sensor. */
  int16_t  z;   /**< Z coordinate of the measured 3D point. Unit is determined by data3d_type in the coordinate system connected with the sensor. */
};

#ifdef __cplusplus
static_assert(sizeof(MkEFrameItem1) == 8, "Incorrect compilation size: MkEFrameItem1");
static_assert(offsetof(MkEFrameItem1, uid) == 0, "Incorrect alignment (MkEFrameItem1, uid)");
static_assert(offsetof(MkEFrameItem1, x) == 2, "Incorrect alignment (MkEFrameItem1, x)");
static_assert(offsetof(MkEFrameItem1, y) == 4, "Incorrect alignment (MkEFrameItem1, y)");
static_assert(offsetof(MkEFrameItem1, z) == 6, "Incorrect alignment (MkEFrameItem1, z)");
#endif

/**
 * @brief Inner structure of the frame item type `MKE_FRAME_TYPE_2`
 *
 */
struct MkEFrameItem2 {
  uint16_t uid; /**< Universal ID key of the detetction */
  int16_t  x;   /**< X coordinate of the measured 3D point. Unit is determined by data3d_type in the coordinate system connected with the sensor. */
  int16_t  y;   /**< X coordinate of the measured 3D point. Unit is determined by data3d_type in the coordinate system connected with the sensor. */
  int16_t  z;   /**< Z coordinate of the measured 3D point. Unit is determined by data3d_type in the coordinate system connected with the sensor. */
  uint16_t lid; /**< Topological point identifier-Reserved for future use */
  uint16_t did; /**< Topological point identifier-Reserved for future use */
};

#ifdef __cplusplus
static_assert(sizeof(MkEFrameItem2) == 12, "Incorrect compilation size: MkEFrameItem2");
static_assert(offsetof(MkEFrameItem2, uid) == 0, "Incorrect alignment (MkEFrameItem2, uid)");
static_assert(offsetof(MkEFrameItem2, x) == 2, "Incorrect alignment (MkEFrameItem2, x)");
static_assert(offsetof(MkEFrameItem2, y) == 4, "Incorrect alignment (MkEFrameItem2, y)");
static_assert(offsetof(MkEFrameItem2, z) == 6, "Incorrect alignment (MkEFrameItem2, z)");
static_assert(offsetof(MkEFrameItem2, lid) == 8, "Incorrect alignment (MkEFrameItem2, lid)");
static_assert(offsetof(MkEFrameItem2, did) == 10, "Incorrect alignment (MkEFrameItem2, did)");
#endif

#ifndef MKE_LARGEST_FRAME_TYPE
    #define MKE_LARGEST_FRAME_TYPE mke::api::MkEFrameItem2
#endif

// =============================================================================

/** @}*/

#undef _IN_MKEAPI_H_

#if defined(__cplusplus) && !defined(MKEAPI_C)
} // namespace api
} // namespace mke
#endif

#endif // _MKEAPI_H_
