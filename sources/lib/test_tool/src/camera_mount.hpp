#ifndef CAMERA_MOUNT_HPP
#define CAMERA_MOUNT_HPP

#include "camera.hpp"
#include <opencv2/core.hpp>

template <typename FltT>
class CameraMount
{
public:
	typedef cv::Point3_<FltT> Point3;
	typedef cv::Point_<FltT> Point2;
	typedef cv::Vec<FltT, 3> Vec3;
	typedef cv::Matx<FltT, 3, 3> Mat33;
	typedef cv::Matx<FltT, 4, 4> Mat44;

	explicit CameraMount(const Camera<FltT> &cam);
	~CameraMount();

	void setExtrinsics(const Point3 &pos, const Vec3 &rot);
	inline Mat33 getRotationMatrix() const { return R_cam2mnt_; }
	inline Vec3 getTranslationVector() const { return t_cam2mnt_; }
	inline const Camera<FltT> &getCamera() const { return cam_; }

	bool cvtImage2WorldPt(const Point2 &pt2d, Point3 &pt3d, const FltT &depth) const;
	bool cvtWorld2ImagePt(const Point3 &pt3d, Point2 &pt2d) const;
	
private:
	const Camera<FltT> &cam_;
	Mat33 R_cam2mnt_;
	Mat33 R_mnt2cam_;
	Point3 t_cam2mnt_;	
};


#endif