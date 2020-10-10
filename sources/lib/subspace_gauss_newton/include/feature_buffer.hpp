#ifndef FEATURE_BUFFER_HPP
#define FEATURE_BUFFER_HPP

#include "keypoint.hpp"
#include "track.hpp"
#include <string>

struct FeatureBuffer
{
	const KeypointVec& keypoints;
	const TrackVec& tracks;

	explicit FeatureBuffer(
		const KeypointVec& _keypoints,
		const TrackVec& _tracks
	)
		: keypoints(_keypoints)
		, tracks(_tracks)
	{}

	std::string str(const i8 verbose_keypoints = 0, const i8 verbose_tracks = 0, const std::string& prefix = "") const;
};

#endif