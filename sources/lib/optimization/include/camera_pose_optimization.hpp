#ifndef CAMERA_POSE_OPTIMIZATION_HPP
#define CAMERA_POSE_OPTIMIZATION_HPP

#include "definition.hpp"
#include "base_optimization.hpp"
#include "keypoint.hpp"
#include "camera_pose_optimization_parameter.hpp"
#include "camera_pose_subspace_gauss_newton.hpp"

//#define  CAMERA_POSE_OPTIMIZATION_CONSOLE_LOG

class CameraPoseOptimization : public BaseOptimization
{
public:
	CameraPoseOptimization();
	~CameraPoseOptimization();

	inline void setParameter(const CameraPoseOptimizationParameter& param) { param_ = param; }
	inline void setCamera(const PerspectiveCamera<OptFltT>& camera) { optimizer_.setCamera(camera); }

	bool clear() override;

	bool compute(
		const KeypointVec& keypoints,
		const TrackVec& tracks,
		const FrameId oldest_frame_id,
		const FrameId latest_frame_id,
		const PointVec& points,
		CameraPoseVec& camera_poses
	);

private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	void expandWorldBuffer_(
		WorldBuffer& world_buff,
		const FrameId new_frame_id
	);

	CameraPoseOptimizationParameter param_;
	CameraPoseSubspaceGaussNewton<OptFltT> optimizer_;
};

#endif