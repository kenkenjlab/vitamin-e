#include "camera_pose.hpp"
#include "mat_util.hpp"
#include <sstream>

CameraPose::CameraPose(const FrameId _frame_id)
	: frame_id(_frame_id)
	, rotation(0.0, 0.0, 0.0)
	, translation(0.0, 0.0, 0.0)
{}

CameraPose::CameraPose(const FrameId _frame_id, const Vec3& _rotation, const Vec3& _translation)
	: frame_id(_frame_id)
	, rotation(_rotation)
	, translation(_translation)
{}

CameraPose::CameraPose(const FrameId _frame_id, const Mat33& _R, const Vec3& _translation)
	: frame_id(_frame_id)
	, rotation(0.0, 0.0, 0.0)
	, translation(_translation)
{
	MatUtil<CamPoseFltT>::genAngle(_R, rotation);
}

void CameraPose::setRotationMatrix(const Mat33& R)
{
	MatUtil<CamPoseFltT>::genAngle(R, rotation);
}

Mat33 CameraPose::getRotationMatrix() const
{
	Mat33 R;
	MatUtil<CamPoseFltT>::genRotationMatrix(rotation, R);
	return R;
}

Mat44 CameraPose::getTransformationMatrix() const
{
	Mat44 T(Mat44::eye());
	Mat33 R = getRotationMatrix();
	T.val[0] = R.val[0];
	T.val[1] = R.val[1];
	T.val[2] = R.val[2];
	T.val[3] = translation(0);
	T.val[4] = R.val[3];
	T.val[5] = R.val[4];
	T.val[6] = R.val[5];
	T.val[7] = translation(1);
	T.val[8] = R.val[6];
	T.val[9] = R.val[7];
	T.val[10] = R.val[8];
	T.val[11] = translation(2);
	return T;
}

std::string CameraPose::str(const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "FrameId: " << frame_id << ", Pos: " << translation.t() << "[m], Rot: " << rotation.t() << "[rad]";
	return ss.str();
}

cv::Vec3f CameraPose::getCvVec3f() const
{
	return cv::Vec3f(
		static_cast<f32>(translation(0)),
		static_cast<f32>(translation(1)),
		static_cast<f32>(translation(2))
	);
}

cv::Vec3d CameraPose::getCvVec3d() const
{
	return cv::Vec3d(
		static_cast<f64>(translation(0)),
		static_cast<f64>(translation(1)),
		static_cast<f64>(translation(2))
	);
}

cv::Point3f CameraPose::getCvPoint3f() const
{
	return cv::Point3f(
		static_cast<f32>(translation(0)),
		static_cast<f32>(translation(1)),
		static_cast<f32>(translation(2))
	);
}

cv::Point3d CameraPose::getCvPoint3d() const
{
	return cv::Point3d(
		static_cast<f64>(translation(0)),
		static_cast<f64>(translation(1)),
		static_cast<f64>(translation(2))
	);
}