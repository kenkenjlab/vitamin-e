#include "keypoint.hpp"
#include <sstream>

std::string Keypoint::str(const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "Frame-KeypointId: " << frame_id << "-" << keypoint_id << ", Coord: " << coord;
	return ss.str();
}