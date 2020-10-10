#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <opencv2/core.hpp>

template<typename FltT>
struct ImagePointState
{
	typedef cv::Point_<FltT> Point2;

	u64 index;
	u32 index_world;
	Point2 coord;	// [px]
};

template<typename FltT>
struct CameraState
{
	typedef cv::Point3_<FltT> Point3;
	typedef cv::Vec<FltT, 3> Vec3;

	u64 index;
	Point3 position;	// [m]
	Vec3 rotation;		// [rad]
	std::vector<ImagePointState<FltT> > img_points;		// [px]
};

template<typename FltT>
struct WorldPointState
{
	typedef cv::Point3_<FltT> Point3;

	u64 index;
	Point3 coord;	// [m]
};
template<typename FltT>
struct Environment
{
	std::vector<WorldPointState<FltT> > world_points;
	std::vector<CameraState<FltT> > camera_states;
};

#endif