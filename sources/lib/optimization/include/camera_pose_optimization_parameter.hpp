#ifndef CAMERA_POSE_OPTIMIZATION_PARAMETER_HPP
#define CAMERA_POSE_OPTIMIZATION_PARAMETER_HPP

#include "abstract_parameter.hpp"

struct CameraPoseOptimizationParameter : public AbstractParameter
{
	bool isValid() const override;
};

#endif