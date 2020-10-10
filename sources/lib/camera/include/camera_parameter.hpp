#ifndef CAMERA_PARAMETER_HPP
#define CAMERA_PARAMETER_HPP

#include "definition.hpp"
#include "abstract_parameter.hpp"

struct CameraParameter : public AbstractParameter
{
	u32 width, height;

	CameraParameter();

	bool isValid() const override;
};

#endif