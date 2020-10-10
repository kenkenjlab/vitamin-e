#include "point_optimization.hpp"

#ifdef POINT_OPTIMIZATION_CONSOLE_LOG
#include <iostream>
#endif

PointOptimization::PointOptimization()
{
	static_cast<void>(this->clear());
}

PointOptimization::~PointOptimization()
{
	static_cast<void>(this->finalize());
}

bool PointOptimization::clear()
{
	return true;
}

bool PointOptimization::compute(
	const KeypointVec& keypoints,
	const TrackVec& tracks,
	const FrameId oldest_frame_id,
	const FrameId latest_frame_id,
	const CameraPoseVec& camera_poses,
	PointVec& points
)
{
	// Prepare buffers
	FeatureBuffer feature_buff(keypoints, tracks);
	WorldBuffer world_buff(camera_poses, points);
	expandWorldBuffer_(tracks, world_buff);

	// Prepare target indices of camera poses to be optimized
	TargetIndices target_indices;
	bool ret = generateTargetIndices_(tracks, oldest_frame_id, latest_frame_id, target_indices);

	if (ret)
	{
		// Optimize
		ret = optimizer_.compute(feature_buff, target_indices, world_buff);

		// Set status
		for (auto& point : world_buff.points)
		{
			if (point.error < param_.min_err)
			{
				point.status = Point::Status::OK;
			}
			else
			{
				point.status = Point::Status::TOO_BIG_ERROR;
			}
		}

		// Swap
		if (ret)
		{
			std::swap(points, world_buff.points);
		}
	}

	return ret;
}

bool PointOptimization::initializeImpl_()
{
	// Check if parameter is valid
	return param_.isValid();
}

bool PointOptimization::finalizeImpl_()
{
	return true;
}


void PointOptimization::expandWorldBuffer_(
	const TrackVec& tracks,
	WorldBuffer& world_buff
)
{
	// Create new camera pose and point instances
	for (const auto& track : tracks)
	{
		auto point = std::find_if(world_buff.points.begin(), world_buff.points.end(),
			[&track](const Point& pt) { return (pt.track_id == track.track_id); });
		if (point == world_buff.points.end())
		{
			Point point_new;
			point_new.track_id = track.track_id;
			world_buff.points.push_back(point_new);
		}

		for (const auto& kp_id : track.keypoint_ids)
		{
			kp_id;
		}
	}
}