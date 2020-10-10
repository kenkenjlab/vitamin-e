#ifndef CALIB_LOADERR_HPP
#define CALIB_LOADERR_HPP

#include <string>
#include "perspective_camera_parameter.hpp"

class CalibLoader
{
public:
	static bool load(const std::string file_path, PerspectiveCameraParameter &param);

private:

};

#endif