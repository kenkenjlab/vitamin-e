#include "feature_buffer.hpp"
#include <sstream>

std::string FeatureBuffer::str(const i8 verbose_keypoints, const i8 verbose_tracks, const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "#Keypoints: " << keypoints.size() << ", #Tracks: " << tracks.size();
	if (verbose_keypoints > 0)
	{
		for (const auto& keypoint : keypoints)
		{
			ss << std::endl << prefix << keypoint.str("  ");
		}

	}
	if (verbose_tracks > 0)
	{
		for (const auto& track : tracks)
		{
			const bool verbose = (verbose_tracks > 1);
			ss << std::endl << prefix << track.str(verbose, "  ");
		}
	}
	return ss.str();
}