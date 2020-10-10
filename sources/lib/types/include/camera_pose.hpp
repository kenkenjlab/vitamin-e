#ifndef CAMERA_POSE_HPP
#define CAMERA_POSE_HPP

#include "definition.hpp"
#include <vector>
#include <string>

struct CameraPose
{
	FrameId frame_id;
	Vec3 rotation;
	Vec3 translation;

	CameraPose()
		: frame_id(INVALID_FRAME_ID)
		, rotation(0.0, 0.0, 0.0)
		, translation(0.0, 0.0, 0.0)
	{}

	explicit CameraPose(const FrameId _frame_id);

	explicit CameraPose(const FrameId _frame_id, const Vec3& _rotation, const Vec3& _translation);

	explicit CameraPose(const FrameId _frame_id, const Mat33& _R, const Vec3& _translation);
	
	void setRotationMatrix(const Mat33& R);

	Mat33 getRotationMatrix() const;

	Mat44 getTransformationMatrix() const;

	std::string str(const std::string& prefix = "") const;

	cv::Vec3f getCvVec3f() const;

	cv::Vec3d getCvVec3d() const;

	cv::Point3f getCvPoint3f() const;

	cv::Point3d getCvPoint3d() const;
};

using CameraPoseVec = std::vector<CameraPose>;

#endif