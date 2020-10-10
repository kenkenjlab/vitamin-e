#ifndef POINT_OPTIMIZATION_HPP
#define POINT_OPTIMIZATION_HPP

#include "definition.hpp"
#include "base_optimization.hpp"
#include "keypoint.hpp"
#include "point_optimization_parameter.hpp"
#include "point_subspace_gauss_newton.hpp"

//#define POINT_OPTIMIZATION_CONSOLE_LOG

class PointOptimization : public BaseOptimization
{
public:
	PointOptimization();
	~PointOptimization();

	inline void setParameter(const PointOptimizationParameter& param) { param_ = param; }
	inline void setCamera(const PerspectiveCamera<OptFltT>& camera) { optimizer_.setCamera(camera); }

	bool clear() override;

	bool compute(
		const KeypointVec& keypoints,
		const TrackVec& tracks,
		const FrameId oldest_frame_id,
		const FrameId latest_frame_id,
		const CameraPoseVec& camera_poses,
		PointVec& points
	);

private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	void expandWorldBuffer_(
		const TrackVec& tracks,
		WorldBuffer& world_buff
	);

	PointOptimizationParameter param_;
	PointSubspaceGaussNewton<OptFltT> optimizer_;
};

#endif