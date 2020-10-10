#include "environment_builder.hpp"
#include "mat_util.hpp"
#include <ctime>

template <typename FltT>
EnvironmentBuilder<FltT>::EnvironmentBuilder(const CameraMount<FltT> &cam_mount)
	: num_wld_pt_(0u)
	, cam_mount_(cam_mount)
	, num_cam_(0u)
	, fps_(0.0f)
{

}

template <typename FltT>
EnvironmentBuilder<FltT>::~EnvironmentBuilder()
{

}

template <typename FltT>
void EnvironmentBuilder<FltT>::setROI(const Point3 &roi_min, const Point3 &&roi_max)
{
	roi_min_.x = std::min(roi_min.x, roi_max.x);
	roi_min_.y = std::min(roi_min.y, roi_max.y);
	roi_min_.z = std::min(roi_min.z, roi_max.z);
	roi_max_.x = std::max(roi_min.x, roi_max.x);
	roi_max_.y = std::max(roi_min.y, roi_max.y);
	roi_max_.z = std::max(roi_min.z, roi_max.z);
}

template <typename FltT>
bool EnvironmentBuilder<FltT>::build(Environment<FltT> &env) const
{
	// Allocate
	std::vector<CameraState<FltT> > &cam_states = env.camera_states;
	std::vector<WorldPointState<FltT> > &world_points = env.world_points;
	std::vector<Mat33> Rs_cam2wld(num_cam_);
	cam_states.resize(num_cam_);
	world_points.resize(num_wld_pt_);

	// Populate camera positions
	const f32 period_s = 1.0f / fps_;
	f32 elapsed_time_s = 0.0f;
	const Mat33 R_cam2mnt = cam_mount_.getRotationMatrix();
	const Vec3 t_cam2mnt = cam_mount_.getTranslationVector();
	for (size_t idx_cam = 0; idx_cam < num_cam_; ++idx_cam)
	{
		CameraState<FltT> &cam_state = cam_states[idx_cam];
		Mat33 &R_cam2wld = Rs_cam2wld[idx_cam];

		cam_state.index = idx_cam;
		const Vec3 rad_mnt2wld = init_cam_rot_ + ((cam_rot_velo_rad_s_ + cam_rot_acc_rad_s2_ * elapsed_time_s) * elapsed_time_s);
		Mat33 R_mnt2wld;
		MatUtil<FltT>::genRotationMatrix(rad_mnt2wld, R_mnt2wld);
		const Vec3 t_mnt2wld = init_cam_pos_ + Point3((cam_pos_velo_m_s_ + cam_pos_acc_m_s2_ * elapsed_time_s) * elapsed_time_s);
		cam_state.position = R_mnt2wld * t_cam2mnt + t_mnt2wld;
		R_cam2wld = R_mnt2wld * R_cam2mnt;
		MatUtil<FltT>::genAngle(R_cam2wld, cam_state.rotation);
		cam_state.img_points.reserve(num_wld_pt_);

		// Prepare for next iteration
		elapsed_time_s += period_s;
	}

	// Populate world points
	Point3 roi_width = roi_max_ - roi_min_;
	size_t idx_wld_pt = 0;
	const Camera<FltT> &cam = cam_mount_.getCamera();
	while(idx_wld_pt < num_wld_pt_)
	{
		WorldPointState<FltT> &wld_pt = world_points[idx_wld_pt];
		Point3 &pt3d = wld_pt.coord;
		wld_pt.index = idx_wld_pt;
		pt3d.x = roi_min_.x + (roi_width.x) * getUniformRandomVal_();
		pt3d.y = roi_min_.y + (roi_width.y) * getUniformRandomVal_();
		pt3d.z = roi_min_.z + (roi_width.z) * getUniformRandomVal_();

		bool check = true;
		std::vector<ImagePointState<FltT> > img_pts(num_cam_);
		for (size_t idx_cam = 0; idx_cam < num_cam_; ++idx_cam)
		{
			// Check whether it is visible from each camera
			CameraState<FltT> &cam_state = cam_states[idx_cam];
			const Mat33 &R_cam2wld = Rs_cam2wld[idx_cam];
			const Vec3 &t_cam2wld = cam_state.position;
			const Vec3 ray3d = R_cam2wld.t() * (Vec3(pt3d) - t_cam2wld);
			const FltT ray3d_array[3] = { ray3d(0), ray3d(1), ray3d(2) };
			FltT pt2d_array[2] = { 0 };
			if (!cam.cvtCamRay2ImagePt(ray3d_array, pt2d_array))
			{
				// Discard current world point and try again
				check = false;
				break;
			}
			img_pts[idx_cam].coord = Point2(pt2d_array[0], pt2d_array[1]);
		}

		if (check)
		{
			for (size_t idx_cam = 0; idx_cam < num_cam_; ++idx_cam)
			{
				// Populate image points
				CameraState<FltT> &cam_state = cam_states[idx_cam];
				ImagePointState<FltT>& img_pt = img_pts[idx_cam];
				img_pt.index = cam_state.img_points.size();
				img_pt.index_world = static_cast<u32>(idx_wld_pt);
				cam_state.img_points.push_back(img_pt);
			}
			++idx_wld_pt;
		}
	}

	return true;
}

template class EnvironmentBuilder<f32>;
template class EnvironmentBuilder<f64>;