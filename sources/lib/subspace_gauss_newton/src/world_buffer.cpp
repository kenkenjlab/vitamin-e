#include "world_buffer.hpp"
#include <sstream>

std::string WorldBuffer::str(const bool verbose_cameras, const bool verbose_points, const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "#Cam: " << camera_poses.size() << ", #Pts: " << points.size();
	if (verbose_cameras)
	{
		for (const auto& cam_pose : camera_poses)
		{
			ss << std::endl << prefix << cam_pose.str("  ");
		}
	}

	if (verbose_points)
	{
		for (const auto& point : points)
		{
			ss << std::endl << prefix << point.str("  ");
		}
	}
	return ss.str();
}