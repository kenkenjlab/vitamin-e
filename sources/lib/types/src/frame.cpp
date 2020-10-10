#include "frame.hpp"
#include <sstream>

std::string Frame::str(const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "FrameId: " << frame_id << ", Timestamp: " << timestamp << "[s]";
	return ss.str();
}