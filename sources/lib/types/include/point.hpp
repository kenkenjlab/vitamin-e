#ifndef POINT_HPP
#define POINT_HPP

#include "definition.hpp"
#include <vector>
#include <string>
#include <opencv2/core.hpp>

struct Point
{
	enum class Status : int
	{
		UNDEFINED,
		OK,
		TOO_BIG_ERROR,
		BEHIND_CAMERA,
	};

	TrackId track_id;
	Vec3 coord;
	u32 votes;
	OptFltT error;
	Status status;

	Point()
		: track_id(INVALID_TRACK_ID)
		, coord(0.0f, 0.0f, 0.0f)
		, votes(0)
		, error(std::numeric_limits<OptFltT>::max())
		, status(Status::UNDEFINED)
	{}

	std::string str(const std::string& prefix = "") const;

	cv::Vec3f getCvVec3f() const;

	cv::Vec3d getCvVec3d() const;

	cv::Point3f getCvPoint3f() const;

	cv::Point3d getCvPoint3d() const;
};

using PointVec = std::vector<Point>;

#endif