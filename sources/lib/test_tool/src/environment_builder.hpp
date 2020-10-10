#ifndef ENVIRONMENT_BUILDER_HPP
#define ENVIRONMENT_BUILDER_HPP

#include "definition.hpp"
#include "environment.hpp"
#include "camera_mount.hpp"

template <typename FltT>
class EnvironmentBuilder
{
public:
	typedef cv::Point_<FltT> Point2;
	typedef cv::Point3_<FltT> Point3;
	typedef cv::Vec<FltT, 3> Vec3;
	typedef cv::Matx<FltT, 3, 3> Mat33;

	explicit EnvironmentBuilder(const CameraMount<FltT> &cam_mount);
	~EnvironmentBuilder();

	inline void setNumOfWorldPoints(const size_t &num) { num_wld_pt_ = num; }
	void setROI(const Point3 &roi_min, const Point3 &&roi_max);
	inline void setNumOfCameras(const size_t&num) { num_cam_ = num; }
	inline void setFps(const f32 &fps) { fps_ = fps; }
	inline void setInitCamPosition(const Point3 &position) { init_cam_pos_ = position; }
	inline void setInitCamRotation(const Vec3 &rotation) { init_cam_rot_ = rotation; }
	inline void setCamPosVelocity(const Vec3 &velo_m_s) { cam_pos_velo_m_s_ = velo_m_s; }
	inline void setCamPosAcceleration(const Vec3 &acc_m_s2) { cam_pos_acc_m_s2_ = acc_m_s2; }
	inline void setCamRotVelocity(const Vec3 &velo_rad_s) { cam_rot_velo_rad_s_ = velo_rad_s; }
	inline void setCamRotAcceleration(const Vec3 &acc_rad_s2) { cam_rot_acc_rad_s2_ = acc_rad_s2; }

	bool build(Environment<FltT> &env) const;

private:
	// Environment parameters
	size_t num_wld_pt_;
	Point3 roi_min_;	// [m] @ World coordinate
	Point3 roi_max_;	// [m] @ World coordinate

	// Camera parameters
	const CameraMount<FltT> &cam_mount_;
	size_t num_cam_;
	f32 fps_;
	Point3 init_cam_pos_;	// [m] @ World coordinate
	Vec3 init_cam_rot_;
	Vec3 cam_pos_velo_m_s_;		// [m/s]
	Vec3 cam_pos_acc_m_s2_;		// [m/s^2]
	Vec3 cam_rot_velo_rad_s_;	// [rad/s]
	Vec3 cam_rot_acc_rad_s2_;	// [rad/s^s]

	static f32 getUniformRandomVal_(void) { return static_cast<f32>(std::rand()) / RAND_MAX; }
};

#endif