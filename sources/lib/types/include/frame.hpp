#ifndef FRAME_HPP
#define FRAME_HPP

#include "definition.hpp"

struct Frame
{
	Image img;
	FrameId frame_id;
	Timestamp timestamp;

	Frame()
		: frame_id(INVALID_FRAME_ID)
		, timestamp(INVALID_TIMESTAMP)
	{}

	inline bool empty() const { return img.empty(); }

	std::string str(const std::string& prefix = "") const;
};

#endif