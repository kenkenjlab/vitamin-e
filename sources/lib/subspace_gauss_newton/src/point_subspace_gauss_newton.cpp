#include "point_subspace_gauss_newton.hpp"

template <typename FltT>
PointSubspaceGaussNewton<FltT>::PointSubspaceGaussNewton()
{

}

template <typename FltT>
PointSubspaceGaussNewton<FltT>::~PointSubspaceGaussNewton()
{

}

template <typename FltT>
bool PointSubspaceGaussNewton<FltT>::solveSubspace_(const cv::Mat_<FltT> &H, const cv::Mat_<FltT> &g, const size_t idx, const VecMax &init_update, VecMax &final_update) const
{
	const i32 num_elem_cam = static_cast<i32>(OPTIMIZER_MAX_NUM_CAM * 6);
	const i32 idx_mat = num_elem_cam + static_cast<i32>(idx) * 3;
	const cv::Mat_<FltT> gp = g(cv::Rect(0, idx_mat, 1, 3));		// 3x1 vector
	const cv::Mat_<FltT> Hpp_tgt = H(cv::Rect(idx_mat, idx_mat, 3, 3));		// 3x3 matrix

	// Compute right side of equation
	cv::Mat_<FltT> rhs = -gp;		// 3x1 vector
	for (i32 i = 0; i < static_cast<i32>(OPTIMIZER_MAX_NUM_PT); ++i)
	{
		if (i == idx_mat)
		{
			continue;
		}
		const i32 matidx_pt3d = static_cast<i32>(num_elem_cam + i * 3u);
		const cv::Mat_<FltT> Hpp_other = H(cv::Rect(matidx_pt3d, matidx_pt3d, 3, 3));		// 3x3 matrix
		const cv::Mat_<FltT> delta_p_other = cv::Mat_<FltT>(init_update)(cv::Rect(0u, matidx_pt3d, 1u, 3u));		// 3x1 vector
		rhs -= Hpp_other * delta_p_other;
	}
	for (i32 i = 0; i < static_cast<i32>(OPTIMIZER_MAX_NUM_CAM); ++i)
	{
		const i32 matidx_cam = i * 6;
		const cv::Mat_<FltT> HcpT_other = H(cv::Rect(matidx_cam, idx_mat, 6, 3));		// 3x6 matrix
		const cv::Mat_<FltT> delta_c_other = cv::Mat_<FltT>(init_update)(cv::Rect(0, matidx_cam, 1, 6));	// 6x1 vector
		rhs -= HcpT_other * delta_c_other;
	}

	// Solve
	cv::Mat_<FltT> partial_update;
	const bool ret = cv::solve(Hpp_tgt, rhs, partial_update, cv::DECOMP_SVD);
	for (i32 i = 0; i < 3; ++i)
	{
		final_update(idx_mat + i) = partial_update(i);
	}

#ifdef POINT_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
	std::cout << "Hpp_tgt:" << std::endl << Hpp_tgt << std::endl;
	std::cout << "rhs:" << std::endl << rhs << std::endl;
	std::cout << "init_update.t():" << std::endl << init_update.t() << std::endl;
	std::cout << "partial_update.t():" << std::endl << partial_update.t() << std::endl;
	std::cout << "final_update.t():" << std::endl << final_update.t() << std::endl;
#endif

	return ret;
}

// Explicit instantiations
template class PointSubspaceGaussNewton<OptFltT>;