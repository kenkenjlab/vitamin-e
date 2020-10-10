#ifndef CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_HPP
#define CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_HPP

#include "reproj_subspace_gauss_newton.hpp"

//#define CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG

#ifdef CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
#include <iostream>
#endif

#define CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_BASE_CLASS ReprojSubspaceGaussNewton<FltT, OPTIMIZER_MAX_NUM_CAM, OPTIMIZER_MAX_NUM_PT, 0u, OPTIMIZER_MAX_NUM_CAM>

template <typename FltT>
class CameraPoseSubspaceGaussNewton : public CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_BASE_CLASS
{
public:
	using DataT = typename CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::DataT;
	using GuessT = typename CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::GuessT;
	using IndicesT = typename CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::IndicesT;
	using VecMax = typename CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::VecMax;

	CameraPoseSubspaceGaussNewton();
	~CameraPoseSubspaceGaussNewton();

protected:
	bool solveSubspace_(const cv::Mat_<FltT> &H, const cv::Mat_<FltT> &g, const size_t idx, const VecMax &init_update, VecMax &final_update) const override;
};

#endif