#ifndef TRACK_HPP
#define TRACK_HPP

#include "definition.hpp"
#include <vector>
#include <string>

struct Track
{
	TrackId track_id;
	std::vector<KeypointId> keypoint_ids;

	Track()
		: track_id(INVALID_TRACK_ID)
	{}

	explicit Track(const TrackId _track_id, const KeypointId _first_kp_id, const KeypointId _second_kp_id)
		: track_id(_track_id)
		, keypoint_ids(2)
	{
		keypoint_ids[0] = _first_kp_id;
		keypoint_ids[1] = _second_kp_id;
	}

	std::string str(const bool verbose = false, const std::string& prefix = "") const;
};

using TrackVec = std::vector<Track>;

#endif