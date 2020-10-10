#ifndef WORLD_BUFFER_HPP
#define WORLD_BUFFER_HPP

#include "point.hpp"
#include "camera_pose.hpp"

struct WorldBuffer
{
	CameraPoseVec camera_poses;
	PointVec points;

	explicit WorldBuffer(
		const CameraPoseVec& _camera_poses,
		const PointVec& _points
	)
		: camera_poses(_camera_poses)
		, points(_points)
	{}

	std::string str(const bool verbose_cameras = false, const bool verbose_points = false, const std::string& prefix = "") const;
};

#endif