#ifndef TARGET_INDICES_HPP
#define TARGET_INDICES_HPP

#include "definition.hpp"
#include <vector>

struct TargetIndices
{
	std::vector<FrameId> target_frame_ids;
	std::vector<TrackId> target_track_ids;

	inline bool empty() const { return ((target_frame_ids.empty()) || (target_track_ids.empty())); }
};

#endif