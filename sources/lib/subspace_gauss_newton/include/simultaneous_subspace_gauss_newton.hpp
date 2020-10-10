#ifndef SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_HPP
#define SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_HPP

#include "reproj_subspace_gauss_newton.hpp"

//#define SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG

#ifdef SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
#include <iostream>
#endif

#define SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_BASE_CLASS ReprojSubspaceGaussNewton<FltT, OPTIMIZER_MAX_NUM_CAM, OPTIMIZER_MAX_NUM_PT, 0u, OPTIMIZER_MAX_NUM_CAM + OPTIMIZER_MAX_NUM_PT>

template <typename FltT>
class SimultaneousSubspaceGaussNewton : public SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_BASE_CLASS
{
public:
	using DataT = typename SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::DataT;
	using GuessT = typename SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::GuessT;
	using IndicesT = typename SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::IndicesT;
	using VecMax = typename SIMULTANEOUS_SUBSPACE_GAUSS_NEWTON_BASE_CLASS::VecMax;

	SimultaneousSubspaceGaussNewton();
	~SimultaneousSubspaceGaussNewton();

protected:
	bool solveSubspace_(const cv::Mat_<FltT>& H, const cv::Mat_<FltT>& g, const size_t idx, const VecMax& init_update, VecMax& final_update) const override;
	bool solveCameraPoseSubspace_(const cv::Mat_<FltT>& H, const cv::Mat_<FltT>& g, const size_t idx, const VecMax& init_update, VecMax& final_update) const;
	bool solvePointSubspace_(const cv::Mat_<FltT>& H, const cv::Mat_<FltT>& g, const size_t idx, const VecMax& init_update, VecMax& final_update) const;
};

#endif