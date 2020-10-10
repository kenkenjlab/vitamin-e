#ifndef PERSPECTIVE_CAMERA_PARAMETER_HPP
#define PERSPECTIVE_CAMERA_PARAMETER_HPP

#include "definition.hpp"
#include "abstract_parameter.hpp"
#include "camera_parameter.hpp"

struct PerspectiveCameraParameter : public AbstractParameter
{
	CameraParameter base;
	f32 fx, fy, cx, cy;

	PerspectiveCameraParameter();

	bool isValid() const override;
};

#endif