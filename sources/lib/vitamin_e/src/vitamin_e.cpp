#include "vitamin_e.hpp"
#include <iostream>
#include <opencv2/calib3d.hpp>

#ifdef VITAMIN_E_CONSOLE_LOG
#include <iostream>
#endif

VitaminE::VitaminE()
	: phase_(Phase::STARTUP)
	, curr_frame_id_(INVALID_FRAME_ID)
	, prev_frame_id_(INVALID_FRAME_ID)
	, oldest_frame_id_(INVALID_FRAME_ID)
{
	static_cast<void>(this->clear());
}

VitaminE::~VitaminE()
{
	static_cast<void>(this->finalize());
}

void VitaminE::setParameter(const VitaminEParameter& param, i32 width, i32 height)
{
	feat_track_.setParameter(param.feat_track, width, height);
	cam_pose_init_est_.setParameter(param.cam_pose_init_est, param.cam);
	cam_pose_opt_.setParameter(param.cam_pose_opt);
	point_opt_.setParameter(param.point_opt);
	cam_.setParameter(param.cam);
}


bool VitaminE::clear()
{
	bool ret = feat_track_.clear();
	ret &= cam_pose_opt_.clear();
	ret &= point_opt_.clear();

	keypoint_buff_.clear();
	track_buff_.clear();
	cam_pose_buff_.clear();
	point_buff_.clear();

	phase_ = Phase::STARTUP;
	curr_frame_id_ = INVALID_FRAME_ID;
	prev_frame_id_ = INVALID_FRAME_ID;
	oldest_frame_id_ = INVALID_FRAME_ID;

	return ret;
}

bool VitaminE::processOnce(const Frame& frame)
{
	bool ret = false;

	// Save FrameID
	prev_frame_id_ = curr_frame_id_;
	curr_frame_id_ = frame.frame_id;

	// Track feature points
	ret = feat_track_.compute(frame, keypoint_buff_, track_buff_);

	if (phase_ == Phase::STARTUP)
	{
		if (ret)
		{
			phase_ = Phase::GOT_INITIAL_KEYPOINTS;
		}
	}
	else
	{
		// Estimate camera pose
		if (ret)
		{
			if (phase_ == Phase::GOT_INITIAL_KEYPOINTS)
			{
				// Estimate first and second camera poses
				ret = cam_pose_init_est_.compute(keypoint_buff_, track_buff_, prev_frame_id_, curr_frame_id_, cam_pose_buff_);
				if (ret)
				{
					oldest_frame_id_ = prev_frame_id_;
					phase_ = Phase::GOT_INITIAL_CAM_POSES;
				}
			}
			else
			{
				ret = cam_pose_opt_.compute(keypoint_buff_, track_buff_, oldest_frame_id_, curr_frame_id_, point_buff_, cam_pose_buff_);
			}
		}

		// Estimate 3D points
		if (ret)
		{
			ret = point_opt_.compute(keypoint_buff_, track_buff_, oldest_frame_id_, curr_frame_id_, cam_pose_buff_, point_buff_);
		}
	}

	return ret;
}

bool VitaminE::initializeImpl_()
{
	// Initialize camera
	bool ret = cam_.initialize();
	cam_pose_opt_.setCamera(cam_);
	point_opt_.setCamera(cam_);

	// Initialize workers
	ret &= feat_track_.initialize();
	ret &= cam_pose_init_est_.initialize();
	ret &= cam_pose_opt_.initialize();
	ret &= point_opt_.initialize();
	return ret;
}

bool VitaminE::finalizeImpl_()
{
	bool ret = cam_.finalize();
	ret &= feat_track_.finalize();
	ret &= cam_pose_init_est_.finalize();
	ret &= cam_pose_opt_.finalize();
	ret &= point_opt_.finalize();
	return ret;
}