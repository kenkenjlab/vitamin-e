#include "camera_pose_subspace_gauss_newton.hpp"

template <typename FltT>
CameraPoseSubspaceGaussNewton<FltT>::CameraPoseSubspaceGaussNewton()
{

}

template <typename FltT>
CameraPoseSubspaceGaussNewton<FltT>::~CameraPoseSubspaceGaussNewton()
{

}

template <typename FltT>
bool CameraPoseSubspaceGaussNewton<FltT>::solveSubspace_(const cv::Mat_<FltT> &H, const cv::Mat_<FltT> &g, const size_t idx, const VecMax &init_update, VecMax &final_update) const
{
	const i32 num_elem_cam = static_cast<i32>(OPTIMIZER_MAX_NUM_CAM) * 6;
	const i32 idx_mat = static_cast<i32>(idx) * 6;
	const cv::Mat_<FltT> gc = g(cv::Rect(0, idx_mat, 1, 6));		// 6x1 vector
	const cv::Mat_<FltT> Hcc_tgt = H(cv::Rect(idx_mat, idx_mat, 6, 6));		// 6x6 matrix

	// Compute right side of equation
	cv::Mat_<FltT> rhs = -gc;		// 6x1 vector
	for (i32 i = 0u; i < static_cast<i32>(OPTIMIZER_MAX_NUM_CAM); ++i)
	{
		if (i == idx_mat)
		{
			continue;
		}
		const i32 matidx_cam = i * 6;
		const cv::Mat_<FltT> Hcc_other = H(cv::Rect(matidx_cam, matidx_cam, 6, 6));		// 6x6 matrix
		const cv::Mat_<FltT> delta_c_other = cv::Mat_<FltT>(init_update)(cv::Rect(0, matidx_cam, 1, 6));		// 6x1 vector
		rhs -= Hcc_other * delta_c_other;
	}
	for (i32 i = 0u; i < static_cast<i32>(OPTIMIZER_MAX_NUM_PT); ++i)
	{
		const i32 matidx_pt3d = num_elem_cam + i * 3;
		const cv::Mat_<FltT> Hcp_other = H(cv::Rect(matidx_pt3d, idx_mat, 3, 6));		// 6x3 matrix
		const cv::Mat_<FltT> delta_p_other = cv::Mat_<FltT>(init_update)(cv::Rect(0, matidx_pt3d, 1, 3));		// 3x1 vector
		rhs -= Hcp_other * delta_p_other;
	}

	// Solve
	cv::Mat_<FltT> partial_update;
	const bool ret = cv::solve(Hcc_tgt, rhs, partial_update, cv::DECOMP_SVD);
	for (i32 i = 0; i < 6; ++i)
	{
		final_update(idx_mat + i) = partial_update(i);
	}


#ifdef CAMERA_POSE_SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
	std::cout << "Hcc_tgt:" << std::endl << Hcc_tgt << std::endl;
	std::cout << "rhs:" << std::endl << rhs << std::endl;
	std::cout << "init_update.t():" << std::endl << init_update.t() << std::endl;
	std::cout << "partial_update.t():" << std::endl << partial_update.t() << std::endl;
	std::cout << "final_update.t():" << std::endl << final_update.t() << std::endl;
#endif

	return ret;
}

// Explicit instantiations
template class CameraPoseSubspaceGaussNewton<OptFltT>;