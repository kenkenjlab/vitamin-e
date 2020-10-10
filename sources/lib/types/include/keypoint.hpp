#ifndef KEYPOINT_HPP
#define KEYPOINT_HPP

#include "definition.hpp"
#include <vector>
#include <string>
#include <opencv2/core.hpp>

struct Keypoint
{
	KeypointId keypoint_id;
	FrameId frame_id;
	cv::Point2f coord;

	Keypoint()
		: keypoint_id(INVALID_KEYPOINT_ID)
		, frame_id(INVALID_FRAME_ID)
		, coord(INVALID_COORD, INVALID_COORD)
	{}

	explicit Keypoint(const KeypointId _keypoint_id, const FrameId _frame_id, const cv::Point2f &_coord)
		: keypoint_id(_keypoint_id)
		, frame_id(_frame_id)
		, coord(_coord)
	{}

	std::string str(const std::string& prefix = "") const;
};

using KeypointVec = std::vector<Keypoint>;

#endif