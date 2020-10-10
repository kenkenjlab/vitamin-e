#ifndef VITAMIN_E_OPTIMIZATION_PARAMETER_HPP
#define VITAMIN_E_OPTIMIZATION_PARAMETER_HPP

#include "perspective_camera_parameter.hpp"
#include "feature_tracking_parameter.hpp"
#include "camera_pose_initial_estimation_parameter.hpp"
#include "camera_pose_optimization_parameter.hpp"
#include "point_optimization_parameter.hpp"

struct VitaminEParameter : public AbstractParameter
{
	PerspectiveCameraParameter cam;
	FeatureTrackingParameter feat_track;
	CameraPoseInitialEstimationParameter cam_pose_init_est;
	CameraPoseOptimizationParameter cam_pose_opt;
	PointOptimizationParameter point_opt;

	bool isValid() const override;
};

#endif