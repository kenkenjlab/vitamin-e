#ifndef BASE_OPTIMIZATION_HPP
#define BASE_OPTIMIZATION_HPP

#include "definition.hpp"
#include "abstract_worker.hpp"
#include "track.hpp"
#include "world_buffer.hpp"
#include "target_indices.hpp"

//#define BASE_OPTIMIZATION_CONSOLE_LOG

class BaseOptimization : public AbstractWorker
{
public:
	BaseOptimization();
	~BaseOptimization();

protected:
	bool generateTargetIndices_(
		const TrackVec& tracks,
		const FrameId oldest_frame_id,
		const FrameId latest_frame_id,
		TargetIndices& target_indices
	);
};

#endif