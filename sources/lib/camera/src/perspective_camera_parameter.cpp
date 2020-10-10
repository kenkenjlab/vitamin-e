#include "perspective_camera_parameter.hpp"

PerspectiveCameraParameter::PerspectiveCameraParameter()
	: fx(0.f)
	, fy(0.f)
	, cx(-1.f)
	, cy(-1.f)
{

}

bool PerspectiveCameraParameter::isValid() const
{
	// Check in the base class
	bool ret = base.isValid();

	// Check if both fx and fy are non-zero.
	ret &= (std::abs(fx) > std::numeric_limits<decltype(fx)>::epsilon());
	ret &= (std::abs(fy) > std::numeric_limits<decltype(fy)>::epsilon());

	// Check if center point is not negative value.
	ret &= (cx >= 0.f);
	ret &= (cy >= 0.f);

	return ret;
}