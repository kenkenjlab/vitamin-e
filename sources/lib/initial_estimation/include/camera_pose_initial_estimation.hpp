#ifndef CAMERA_POSE_INITIAL_ESTIMATION_HPP
#define CAMERA_POSE_INITIAL_ESTIMATION_HPP

#include "abstract_worker.hpp"
#include "keypoint.hpp"
#include "track.hpp"
#include "camera_pose.hpp"
#include "camera_pose_initial_estimation_parameter.hpp"
#include "perspective_camera_parameter.hpp"

//#define CAMERA_POSE_INITIAL_ESTIMATION_CONSOLE_LOG

class CameraPoseInitialEstimation : public AbstractWorker
{
public:
	CameraPoseInitialEstimation();
	~CameraPoseInitialEstimation();

	inline void setParameter(
		const CameraPoseInitialEstimationParameter& param,
		const PerspectiveCameraParameter& cam_intrinsics
	)
	{
		param_ = param;
		cam_intrinsics_ = cam_intrinsics;
	}

	bool clear() override;

	bool compute(
		const KeypointVec& keypoints,
		const TrackVec& tracks,
		const FrameId prev_frame_id,
		const FrameId curr_frame_id,
		CameraPoseVec& camera_poses
	);

private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	bool findKeypoint_(const KeypointVec& keypoints, const std::vector<KeypointId>& keypoint_ids, const FrameId frame_id, Keypoint& keypoint);

	CameraPoseInitialEstimationParameter param_;
	PerspectiveCameraParameter cam_intrinsics_;
};

#endif