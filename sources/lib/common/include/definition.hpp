#ifndef VITAMINE_DEFINITION_HPP
#define VITAMINE_DEFINITION_HPP

#include <opencv2/core.hpp>

// -----------------------------------------------
// General typedefs

typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef signed long long i64;

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;

typedef float f32;
typedef double f64;

// -----------------------------------------------
// Specific typedefs

typedef i64 FrameId;
typedef i64 KeypointId;
typedef i64 TrackId;
typedef f64 Timestamp;
typedef cv::Mat Image;

typedef f64 CamPoseFltT;
typedef cv::Matx<CamPoseFltT, 3, 3> Mat33;
typedef cv::Matx<CamPoseFltT, 3, 1> Vec3;
typedef cv::Matx<CamPoseFltT, 4, 1> Vec4;
typedef cv::Matx<CamPoseFltT, 4, 4> Mat44;
typedef cv::Matx<CamPoseFltT, 3, 4> Mat34;
typedef cv::Matx<CamPoseFltT, 4, 3> Mat43;

typedef CamPoseFltT OptFltT;

// -----------------------------------------------
// Constants

constexpr FrameId INVALID_FRAME_ID = -1;
constexpr KeypointId INVALID_KEYPOINT_ID = -1;
constexpr TrackId INVALID_TRACK_ID = -1;
constexpr Timestamp INVALID_TIMESTAMP = -1.0;
constexpr f32 INVALID_COORD = -1.0;

constexpr size_t OPTIMIZER_MAX_NUM_CAM = 3u;
constexpr size_t OPTIMIZER_MAX_NUM_PT = 10u;

#endif