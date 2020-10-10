#ifndef VITAMIN_E_HPP
#define VITAMIN_E_HPP

#include "abstract_worker.hpp"
#include "feature_tracking.hpp"
#include "camera_pose_initial_estimation.hpp"
#include "camera_pose_optimization.hpp"
#include "point_optimization.hpp"
#include "perspective_camera.hpp"
#include "vitamin_e_parameter.hpp"

//#define VITAMIN_E_CONSOLE_LOG

class VitaminE : public AbstractWorker
{
public:
	enum class Phase
	{
		STARTUP,
		GOT_INITIAL_KEYPOINTS,
		GOT_INITIAL_CAM_POSES,
	};

	VitaminE();
	~VitaminE();

	void setParameter(const VitaminEParameter& param, i32 width, i32 height);

	bool clear() override;

	bool processOnce(const Frame &frame);

	inline const KeypointVec& getKeypointBuffer() const { return keypoint_buff_; }
	inline const TrackVec& getTrackBuffer() const { return track_buff_; }
	inline const CameraPoseVec& getCameraPoseBuffer() const { return cam_pose_buff_; }
	inline const PointVec& getPointBuffer() const { return point_buff_; }


private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	// Workers
	PerspectiveCamera<f64> cam_;
	FeatureTracking feat_track_;
	CameraPoseInitialEstimation cam_pose_init_est_;
	CameraPoseOptimization cam_pose_opt_;
	PointOptimization point_opt_;

	// Buffers
	KeypointVec keypoint_buff_;
	TrackVec track_buff_;
	CameraPoseVec cam_pose_buff_;
	PointVec point_buff_;

	// Flags
	Phase phase_;
	FrameId curr_frame_id_, prev_frame_id_, oldest_frame_id_;
};

#endif