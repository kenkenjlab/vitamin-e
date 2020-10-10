#include "base_optimization.hpp"

#ifdef BASE_OPTIMIZATION_CONSOLE_LOG
#include <iostream>
#endif

BaseOptimization::BaseOptimization()
{}

BaseOptimization::~BaseOptimization()
{}

bool BaseOptimization::generateTargetIndices_(
	const TrackVec& tracks,
	const FrameId oldest_frame_id,
	const FrameId latest_frame_id,
	TargetIndices& target_indices
)
{
	// Prepare target indices of camera poses to be optimized
	for (FrameId frame_id = latest_frame_id;
		(frame_id >= oldest_frame_id)
		&& (target_indices.target_frame_ids.size() < OPTIMIZER_MAX_NUM_CAM);
		--frame_id)
	{
		target_indices.target_frame_ids.push_back(frame_id);
	}

	// Prepare target indices of points to be optimized
	for (auto track = tracks.crbegin();
		(track != tracks.crend())
		&& (target_indices.target_track_ids.size() < OPTIMIZER_MAX_NUM_PT);
		++track)
	{
		target_indices.target_track_ids.push_back(track->track_id);
	}

	return !target_indices.empty();
}