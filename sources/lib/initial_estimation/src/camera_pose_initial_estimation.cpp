#include "camera_pose_initial_estimation.hpp"
#include <opencv2/calib3d.hpp>

#ifdef CAMERA_POSE_INITIAL_ESTIMATION_CONSOLE_LOG
#include <iostream>
#endif

CameraPoseInitialEstimation::CameraPoseInitialEstimation()
{
	static_cast<void>(this->clear());
}

CameraPoseInitialEstimation::~CameraPoseInitialEstimation()
{
	static_cast<void>(this->finalize());
}

bool CameraPoseInitialEstimation::clear()
{
	return true;
}

bool CameraPoseInitialEstimation::compute(
	const KeypointVec& keypoints,
	const TrackVec& tracks,
	const FrameId prev_frame_id,
	const FrameId curr_frame_id,
	CameraPoseVec& camera_poses
)
{
	bool ret = false;
	const size_t num_points = tracks.size();

	// Check number of keypoints
	if (num_points >= 5u)
	{
		std::vector<cv::Point2f> prev_points;
		std::vector<cv::Point2f> curr_points;
		prev_points.reserve(num_points);
		curr_points.reserve(num_points);

		// Extract keypoints in both target two frames
		for (const auto& track : tracks)
		{
			Keypoint prev_kp, curr_kp;
			bool found = findKeypoint_(keypoints, track.keypoint_ids, prev_frame_id, prev_kp);
			found &= findKeypoint_(keypoints, track.keypoint_ids, curr_frame_id, curr_kp);
			if (found)
			{
				prev_points.push_back(prev_kp.coord);
				curr_points.push_back(curr_kp.coord);
			}
		}

		// Generate camera intrinsic matrix (perspective camera)
		Mat33 cam_matrix(Mat33::eye());
		cam_matrix(0, 0) = cam_intrinsics_.fx;
		cam_matrix(1, 1) = cam_intrinsics_.fy;
		cam_matrix(0, 2) = cam_intrinsics_.cx;
		cam_matrix(1, 2) = cam_intrinsics_.cy;

		// Estimate essential matrix
		cv::Mat E = cv::findEssentialMat(prev_points, curr_points, cam_matrix, cv::LMEDS, 0.9999, 0.5);
		ret = !E.empty();

		// Extract R and t from essential matrix
		if (ret)
		{
			Mat33 R;
			Vec3 t;
			static_cast<void>(cv::recoverPose(E, prev_points, curr_points, cam_matrix, R, t));
			CameraPose cam_pose_1st(prev_frame_id);
			CameraPose cam_pose_2nd(curr_frame_id, R, t);
			camera_poses.push_back(cam_pose_1st);
			camera_poses.push_back(cam_pose_2nd);

#ifdef CAMERA_POSE_INITIAL_ESTIMATION_CONSOLE_LOG
			std::cout << "R: " << std::endl << R << std::endl;
			std::cout << "t: " << std::endl << t.t() << " (Norm: " << cv::norm(t) << ")" << std::endl;
#endif
		}
	}

	return ret;
}

bool CameraPoseInitialEstimation::initializeImpl_()
{
	// Check if parameter is valid
	bool ret = param_.isValid();
	ret &= cam_intrinsics_.isValid();
	return ret;
}

bool CameraPoseInitialEstimation::finalizeImpl_()
{
	return true;
}

bool CameraPoseInitialEstimation::findKeypoint_(const KeypointVec& keypoints, const std::vector<KeypointId>& keypoint_ids, const FrameId frame_id, Keypoint& keypoint)
{
#if 0
	auto keypoint_id = std::find_if(keypoint_ids.cbegin(), keypoint_ids.cend(),
		[&keypoints, &frame_id](const KeypointId kp_id)
		{
			auto keypoint = std::find_if(keypoints.cbegin(), keypoints.cend(),
				[&frame_id](const Keypoint& kp) { return ((kp.frame_id == frame_id)
					&& (kp.keypoint_id == kp_id)); });
			return (keypoint != keypoints.cend());
		});
#endif

	auto keypoint_cit = std::find_if(keypoints.cbegin(), keypoints.cend(),
		[&keypoint_ids, &frame_id](const Keypoint& kp)
		{
			auto keypoint_id = std::find_if(keypoint_ids.cbegin(), keypoint_ids.cend(),
				[&kp, &frame_id](const KeypointId kp_id)
				{
					return ((kp.frame_id == frame_id)
						&& (kp.keypoint_id == kp_id));
				});
			return (keypoint_id != keypoint_ids.cend());
		});

	const bool ret = (keypoint_cit != keypoints.cend());
	if (ret)
	{
		keypoint = *keypoint_cit;
	}

	return ret;
}