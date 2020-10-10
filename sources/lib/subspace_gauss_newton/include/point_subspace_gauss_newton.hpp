#ifndef POINT_SUBSPACE_GAUSS_NEWTON_HPP
#define POINT_SUBSPACE_GAUSS_NEWTON_HPP

#include "reproj_subspace_gauss_newton.hpp"

//#define POINT_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG

#ifdef POINT_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
#include <iostream>
#endif

#define POINT_SUBSPACE_GAUSS_NEWTON_BASE_CLASS ReprojSubspaceGaussNewton<FltT, OPTIMIZER_MAX_NUM_CAM, OPTIMIZER_MAX_NUM_PT, 0u, OPTIMIZER_MAX_NUM_PT>

template <typename FltT>
class PointSubspaceGaussNewton : public POINT_SUBSPACE_GAUSS_NEWTON_BASE_CLASS
{
public:
	using DataT = typename POINT_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::DataT;
	using GuessT = typename POINT_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::GuessT;
	using IndicesT = typename POINT_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::IndicesT;
	using VecMax = typename POINT_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::VecMax;

	PointSubspaceGaussNewton();
	~PointSubspaceGaussNewton();

protected:
	bool solveSubspace_(const cv::Mat_<FltT> &H, const cv::Mat_<FltT> &g, const size_t idx, const VecMax &init_update, VecMax &final_update) const override;
};

#endif