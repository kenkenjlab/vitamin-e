#include "camera_pose_optimization.hpp"

#ifdef CAMERA_POSE_OPTIMIZATION_CONSOLE_LOG
#include <iostream>
#endif

CameraPoseOptimization::CameraPoseOptimization()
{
	static_cast<void>(this->clear());
}

CameraPoseOptimization::~CameraPoseOptimization()
{
	static_cast<void>(this->finalize());
}

bool CameraPoseOptimization::clear()
{
	return true;
}

bool CameraPoseOptimization::compute(
	const KeypointVec& keypoints,
	const TrackVec& tracks,
	const FrameId oldest_frame_id,
	const FrameId latest_frame_id,
	const PointVec& points,
	CameraPoseVec& camera_poses)
{
	// Prepare buffers
	FeatureBuffer feature_buff(keypoints, tracks);
	WorldBuffer world_buff(camera_poses, points);
	expandWorldBuffer_(world_buff, latest_frame_id);

	// Prepare target indices of camera poses to be optimized
	TargetIndices target_indices;
	bool ret = generateTargetIndices_(tracks, oldest_frame_id, latest_frame_id, target_indices);

	// Optimize
	if (ret)
	{
		ret = optimizer_.compute(feature_buff, target_indices, world_buff);
		if (ret)
		{
			std::swap(camera_poses, world_buff.camera_poses);
		}
	}

	return ret;
}

bool CameraPoseOptimization::initializeImpl_()
{
	// Check if parameter is valid
	return param_.isValid();
}

bool CameraPoseOptimization::finalizeImpl_()
{
	return true;
}

void CameraPoseOptimization::expandWorldBuffer_(
	WorldBuffer& world_buff,
	const FrameId new_frame_id
)
{
	if (world_buff.camera_poses.empty())
	{
		world_buff.camera_poses.push_back(CameraPose(new_frame_id));
	}
	else
	{
		CameraPose new_cam_pose = world_buff.camera_poses.back();
		new_cam_pose.frame_id = new_frame_id;
		world_buff.camera_poses.push_back(new_cam_pose);
	}
}