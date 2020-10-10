#include "camera_parameter.hpp"

CameraParameter::CameraParameter()
	: width(0u)
	, height(0u)
{

}

bool CameraParameter::isValid() const
{
	bool ret = isInitialized();

	// Check if size is positive
	ret &= (width > 0u);
	ret &= (height > 0u);

	return ret;
}