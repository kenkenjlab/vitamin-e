#ifndef REPROJ_SUBSPACE_GAUSS_NEWTON_HPP
#define REPROJ_SUBSPACE_GAUSS_NEWTON_HPP

#include "subspace_gauss_newton.hpp"
#include "feature_buffer.hpp"
#include "world_buffer.hpp"
#include "target_indices.hpp"
#include "perspective_camera.hpp"

//#define REPROJ_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG

#ifdef REPROJ_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
#include <iostream>
#endif

#define REPROJ_SUBSPACE_GAUSS_NEWTON_BASE_CLASS SubspaceGaussNewton<FltT, (MaxNumCam * 6u + MaxNumPt * 3u), IdxBegin, IdxEnd, FeatureBuffer, WorldBuffer, TargetIndices>

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
class ReprojSubspaceGaussNewton : public REPROJ_SUBSPACE_GAUSS_NEWTON_BASE_CLASS
{
public:
	using DataT = FeatureBuffer;
	using GuessT = WorldBuffer;
	using IndicesT = TargetIndices;
	using VecMax = typename REPROJ_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::VecMax;

	ReprojSubspaceGaussNewton();
	~ReprojSubspaceGaussNewton();

	inline void setCamera(const PerspectiveCamera<FltT>& camera) { camera_ = camera; }

	constexpr inline size_t getMaxNumCam() const { return MaxNumCam; }
	constexpr inline size_t getMaxNumPt() const { return MaxNumPt; }

protected:
	bool checkInputValidity_(const DataT &data, const IndicesT& indices, const GuessT &guess) const override;
	cv::Mat_<FltT> objFunc_(const DataT &data, const IndicesT& indices, GuessT &guess) const override;
	bool cvtGuess2Param_(const GuessT &guess, const IndicesT& indices, VecMax &param) const override;
	bool cvtParam2Guess_(const VecMax &param, const IndicesT& indices, GuessT &guess) const override;

	PerspectiveCamera<FltT> camera_;

	struct CameraPoseIntermediate
	{
		typedef cv::Matx<FltT, 3, 3> RotMat33;
		typedef cv::Matx<FltT, 3, 1> TransVec3;

		FrameId frame_id;
		RotMat33 rotation_t;		// Transposed
		TransVec3 translation;		// [m]

		void set(const CameraPose& cam_stat);
	};
	using CameraPoseIntVec = typename std::vector<CameraPoseIntermediate>;
};

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
ReprojSubspaceGaussNewton<FltT, MaxNumCam, MaxNumPt, IdxBegin, IdxEnd>::ReprojSubspaceGaussNewton()
{

}

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
ReprojSubspaceGaussNewton<FltT, MaxNumCam, MaxNumPt, IdxBegin, IdxEnd>::~ReprojSubspaceGaussNewton()
{

}

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
bool ReprojSubspaceGaussNewton<FltT, MaxNumCam, MaxNumPt, IdxBegin, IdxEnd>::checkInputValidity_(const DataT &data, const IndicesT& indices, const GuessT &guess) const
{
	bool ret = (
		(data.tracks.size() == guess.points.size())
		&& (!indices.target_frame_ids.empty())
		&& (!indices.target_track_ids.empty())
		&& (indices.target_frame_ids.size() <= MaxNumCam)
		&& (indices.target_track_ids.size() <= MaxNumPt)
		);
	return ret;
}

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
cv::Mat_<FltT> ReprojSubspaceGaussNewton<FltT, MaxNumCam, MaxNumPt, IdxBegin, IdxEnd>::objFunc_(const DataT &data, const IndicesT& indices, GuessT &guess) const
{
	const size_t size_frame_ids = indices.target_frame_ids.size();
	const size_t size_track_ids = indices.target_track_ids.size();
	cv::Mat_<FltT> error(static_cast<i32>(size_frame_ids * size_track_ids), 1);		// Allocate maximum number of error space to be calculated

	// Generate transposed rotation matrix for each camera
	CameraPoseIntVec cam_pose_ints(size_frame_ids);
	for (size_t i = 0u; i < size_frame_ids; ++i)
	{
		const FrameId frame_id = indices.target_frame_ids[i];
		auto camera_pose = std::find_if(guess.camera_poses.cbegin(), guess.camera_poses.cend(),
			[&frame_id](const CameraPose& cam_pose) { return (cam_pose.frame_id == frame_id); });
		assert(camera_pose != guess.camera_poses.cend() && "Corresponding CameraPose is not found while processing objFunc_(). Either indices or camera poses might be corrupted.");
		cam_pose_ints[i].set(*camera_pose);
	}

	// Evaluate
	i32 index = 0;
	for (size_t i = 0u; i < size_track_ids; ++i)
	{
		// Find corresponding Point
		const TrackId track_id = indices.target_track_ids[i];
		auto cit_track = std::find_if(data.tracks.cbegin(), data.tracks.cend(),
			[&track_id](const Track& track) { return (track.track_id == track_id); });
		assert(cit_track != data.tracks.cend() && "Corresponding Track is not found while processing objFunc_(). Either indices or tracks might be corrupted.");
		auto it_pt = std::find_if(guess.points.begin(), guess.points.end(),
			[&track_id](const Point& point) { return (point.track_id == track_id); });
		if (it_pt == guess.points.end())
		{
			assert(false && "Here Point is not found. Never comes here.");
			continue;
		}

		Point& point = *it_pt;
		const Track& track = *cit_track;
		for (size_t j = 0u; j < track.keypoint_ids.size(); ++j)
		{
			// Find corresponding Keypoint
			const KeypointId &keypoint_id = track.keypoint_ids[j];
			KeypointVec::const_iterator cit_kp = std::find_if(data.keypoints.cbegin(), data.keypoints.cend(),
				[&keypoint_id](const Keypoint &keypoint) { return keypoint.keypoint_id == keypoint_id; });
			if (cit_kp == data.keypoints.cend())
			{
				assert(false && "Here Keypoint is not found. Never comes here.");
				continue;
			}
			const Keypoint &keypoint = *cit_kp;

			// Find corresponding camera
			auto cit_cam = std::find_if(cam_pose_ints.cbegin(), cam_pose_ints.cend(),
				[&keypoint](const CameraPoseIntermediate& cam_pose) { return cam_pose.frame_id == keypoint.frame_id; });
			if (cit_cam == cam_pose_ints.cend())
			{
				// Here specified camera pose is not a target now.
				continue;
			}
			const CameraPoseIntermediate &cam_pose_int = *cit_cam;

			const typename CameraPoseIntermediate::TransVec3 ray3d = cam_pose_int.rotation_t * (point.coord - cam_pose_int.translation);
			const FltT ray3d_ary[3] = { ray3d(0), ray3d(1), ray3d(2) };
			FltT pt2d_theo_ary[2] = { 0 };
			static_cast<void>(camera_.cvtCamRay2ImagePt(ray3d_ary, pt2d_theo_ary));
			const cv::Point2f pt2d_theo(static_cast<f32>(pt2d_theo_ary[0]), static_cast<f32>(pt2d_theo_ary[1]));
			const cv::Point2f res = keypoint.coord - pt2d_theo;		// Vector between the answer(y) point and estimated one (Ax+b).
			const FltT err = this->cast(res.dot(res));	// Squared L2 distance between two points (y and Ax+b)
			//error(index) = this->cast(err) / (err + 1.0);		// Geman-McClure function (M-estimator)
			point.error = err;
			error(index) = err;
			++index;
		}
	}
#ifdef REPROJ_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
	//std::cout << "r:" << std::endl << error.t() << std::endl;
#endif

	return error;
}

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
bool ReprojSubspaceGaussNewton<FltT, MaxNumCam, MaxNumPt, IdxBegin, IdxEnd>::cvtGuess2Param_(const GuessT &guess, const IndicesT &indices, VecMax &param) const
{
	const i32 size_frame_ids = static_cast<i32>(indices.target_frame_ids.size());
	const i32 size_track_ids = static_cast<i32>(indices.target_track_ids.size());
	param = VecMax::zeros();

	for (i32 idx = 0u; idx < size_frame_ids; ++idx)
	{
		const FrameId frame_id = indices.target_frame_ids[idx];
		auto cam_pose = std::find_if(guess.camera_poses.cbegin(), guess.camera_poses.cend(),
			[&frame_id](const CameraPose& cam_pose) { return (cam_pose.frame_id == frame_id); });
		assert(cam_pose != guess.camera_poses.cend() && "Corresponding CameraPose is not found while processing cvtGuess2Param_().");
		const i32 idx6 = idx * 6;
		param(idx6) = cam_pose->rotation(0);
		param(idx6 + 1) = cam_pose->rotation(1);
		param(idx6 + 2) = cam_pose->rotation(2);
		param(idx6 + 3) = cam_pose->translation(0);
		param(idx6 + 4) = cam_pose->translation(1);
		param(idx6 + 5) = cam_pose->translation(2);
	}

	for (i32 idx = 0u; idx < size_track_ids; ++idx)
	{
		const TrackId track_id = indices.target_track_ids[idx];
		auto point = std::find_if(guess.points.begin(), guess.points.end(),
			[&track_id](const Point& point) { return (point.track_id == track_id); });
		assert(point != guess.points.cend() && "Corresponding Point is not found while processing cvtGuess2Param_().");
		const i32 idx3 = size_frame_ids * 6 + idx * 3;
		param(idx3) = point->coord(0);
		param(idx3 + 1) = point->coord(1);
		param(idx3 + 2) = point->coord(2);
	}

	return true;
}

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
bool ReprojSubspaceGaussNewton<FltT, MaxNumCam, MaxNumPt, IdxBegin, IdxEnd>::cvtParam2Guess_(const VecMax &param, const IndicesT& indices, GuessT &guess) const
{
	const i32 size_frame_ids = static_cast<i32>(indices.target_frame_ids.size());
	const i32 size_track_ids = static_cast<i32>(indices.target_track_ids.size());

	for (i32 idx = 0u; idx < size_frame_ids; ++idx)
	{
		const FrameId frame_id = indices.target_frame_ids[idx];
		auto cam_pose = std::find_if(guess.camera_poses.begin(), guess.camera_poses.end(),
			[&frame_id](const CameraPose& cam_pose) { return (cam_pose.frame_id == frame_id); });
		assert(cam_pose != guess.camera_poses.cend() && "Corresponding CameraPose is not found while processing cvtParam2Guess_().");
		const i32 idx6 = idx * 6;
		cam_pose->rotation(0) = param(idx6);
		cam_pose->rotation(1) = param(idx6 + 1);
		cam_pose->rotation(2) = param(idx6 + 2);
		cam_pose->translation(0) = param(idx6 + 3);
		cam_pose->translation(1) = param(idx6 + 4);
		cam_pose->translation(2) = param(idx6 + 5);
	}

	for (i32 idx = 0u; idx < size_track_ids; ++idx)
	{
		const TrackId track_id = indices.target_track_ids[idx];
		auto point = std::find_if(guess.points.begin(), guess.points.end(),
			[&track_id](const Point& point) { return (point.track_id == track_id); });
		assert(point != guess.points.cend() && "Corresponding Point is not found while processing cvtParam2Guess_().");
		const i32 idx3 = size_frame_ids * 6 + idx * 3;
		point->coord(0) = param(idx3);
		point->coord(1) = param(idx3 + 1);
		point->coord(2) = param(idx3 + 2);
	}

	return true;
}

template <typename FltT, size_t MaxNumCam, size_t MaxNumPt, size_t IdxBegin, size_t IdxEnd>
void ReprojSubspaceGaussNewton<FltT, MaxNumCam, MaxNumPt, IdxBegin, IdxEnd>::CameraPoseIntermediate::set(const CameraPose& cam_stat)
{
	frame_id = cam_stat.frame_id;
	translation = cam_stat.translation;
	rotation_t = cam_stat.getRotationMatrix().t();	// Transpose
}

#endif