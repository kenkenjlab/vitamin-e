#ifndef CAMERA_POSE_INITIAL_ESTIMATION_PARAMETER_HPP
#define CAMERA_POSE_INITIAL_ESTIMATION_PARAMETER_HPP

#include "abstract_parameter.hpp"

struct CameraPoseInitialEstimationParameter : public AbstractParameter
{
	bool isValid() const override;
};

#endif