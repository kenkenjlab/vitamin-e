#include "camera_mount.hpp"
#include "mat_util.hpp"

template <typename FltT>
CameraMount<FltT>::CameraMount(const Camera<FltT> &cam)
	: cam_(cam)
	, R_cam2mnt_(Mat33::eye())
	, R_mnt2cam_(Mat33::eye())
{

}

template <typename FltT>
CameraMount<FltT>::~CameraMount()
{

}

template <typename FltT>
void CameraMount<FltT>::setExtrinsics(const Point3 &pos, const Vec3 &rot)
{
	MatUtil<FltT>::genRotationMatrix(rot, R_cam2mnt_);
	R_mnt2cam_ = R_cam2mnt_.t();
	t_cam2mnt_ = pos;
}

template <typename FltT>
bool CameraMount<FltT>::cvtImage2WorldPt(const Point2 &pt2d, Point3 &pt3d, const FltT &depth) const
{
	FltT pt2d_array[2] = { pt2d.x, pt2d.y };
	FltT ray_array[3] = { 0 };
	const bool ret = cam_.cvtImagePt2CamRay(pt2d_array, ray_array, true);
	if (ret)
	{
		pt3d.x = ray_array[0];
		pt3d.y = ray_array[1];
		pt3d.z = ray_array[2];
		pt3d *= depth;
		pt3d = R_cam2mnt_ * pt3d + t_cam2mnt_;
	}

	return ret;
}

template <typename FltT>
bool CameraMount<FltT>::cvtWorld2ImagePt(const Point3 &pt3d, Point2 &pt2d) const
{
	const Point3 pt3d_cam = R_mnt2cam_ * (pt3d - t_cam2mnt_);
	FltT pt3d_array[3] = { pt3d_cam.x, pt3d_cam.y, pt3d_cam.z };
	FltT pt2d_array[2] = { 0 };
	bool ret = cam_.cvtCamRay2ImagePt(pt3d_array, pt2d_array);
	if (ret)
	{
		pt2d.x = pt2d_array[0];
		pt2d.y = pt2d_array[1];
	}
	return ret;
}

template class CameraMount<f32>;
template class CameraMount<f64>;