#include "point.hpp"
#include <sstream>

std::string  Point::str(const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "TrackId: " << track_id << ", Coord: " << coord.t() << ", Error: " << error << ", Status: ";

	switch (status)
	{
	case Status::OK:
		ss << "OK";
		break;
	case Status::TOO_BIG_ERROR:
		ss << "TOO_BIG_ERROR";
		break;
	case Status::UNDEFINED:
		ss << "UNDEFINED";
		break;
	default:
		assert(false && "Unknown status is given. Update the str() method for added status.");
		break;
	}

	return ss.str();
}

cv::Vec3f Point::getCvVec3f() const
{
	return cv::Vec3f(
		static_cast<f32>(coord(0)),
		static_cast<f32>(coord(1)),
		static_cast<f32>(coord(2))
	);
}

cv::Vec3d Point::getCvVec3d() const
{
	return cv::Vec3d(
		static_cast<f64>(coord(0)),
		static_cast<f64>(coord(1)),
		static_cast<f64>(coord(2))
	);
}

cv::Point3f Point::getCvPoint3f() const
{
	return cv::Point3f(
		static_cast<f32>(coord(0)),
		static_cast<f32>(coord(1)),
		static_cast<f32>(coord(2))
	);
}

cv::Point3d Point::getCvPoint3d() const
{
	return cv::Point3d(
		static_cast<f64>(coord(0)),
		static_cast<f64>(coord(1)),
		static_cast<f64>(coord(2))
	);
}