#include "track.hpp"
#include <sstream>

std::string Track::str(const bool verbose, const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "TrackId: " << track_id << ", #KeypointIds: " << keypoint_ids.size();
	if (verbose)
	{
		ss << std::endl << prefix;
		for (const auto& keypoint_id : keypoint_ids)
		{
			ss << keypoint_id << ",";
		}
	}
	return ss.str();
};