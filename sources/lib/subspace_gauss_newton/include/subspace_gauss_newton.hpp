#ifndef SUBSPACE_GAUSS_NEWTON_HPP
#define SUBSPACE_GAUSS_NEWTON_HPP

#include "definition.hpp"
#include <opencv2/core.hpp>

//#define SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG

#ifdef SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
#include <iostream>
#endif

template <typename FltT, size_t DimMax, size_t IdxBegin, size_t IdxEnd, class DataT, class GuessT, class IndicesT>
class SubspaceGaussNewton
{
public:
	using VecMax = cv::Matx<FltT, DimMax, 1>;

	SubspaceGaussNewton();
	~SubspaceGaussNewton();

	inline void setAlpha(const FltT &alpha) { alpha_ = alpha; }
	inline void setTolerance(const FltT &tolerance) { sqr_tolerance_ = tolerance * tolerance; }
	inline void setEpsilon(const VecMax &epsion) { epsilon_ = epsion; epsilon_inv_ = VecMax::ones().div(epsilon_); }
	inline void setMaxIterations(const u32 &max_iter) { max_iter_ = max_iter; }
	bool compute(const DataT &data, const IndicesT& indices, GuessT& guess) const;
	template <typename ValT>
	inline FltT cast(const ValT &val) const { return static_cast<FltT>(val); }

protected:
	bool computeOnce_(const DataT& data, const IndicesT& indices, const size_t idx, GuessT& guess, FltT& norm_update, FltT& ssd) const;
	bool solveNormalEquation_(const cv::Mat_<FltT>& J, const cv::Mat_<FltT>& e, const size_t idx, const VecMax& init_update, VecMax& final_update) const;
	virtual bool checkInputValidity_(const DataT& data, const IndicesT& indices, const GuessT& guess) const = 0;
	virtual cv::Mat_<FltT> objFunc_(const DataT& data, const IndicesT& indices, GuessT& guess) const = 0;		// 'guess' is assumed non-constant argument so that we can update error value into it.
	virtual bool cvtGuess2Param_(const GuessT& guess, const IndicesT& indices, VecMax& param) const = 0;
	virtual bool cvtParam2Guess_(const VecMax& param, const IndicesT& indices, GuessT& guess) const = 0;
	virtual bool solveSubspace_(const cv::Mat_<FltT>& H, const cv::Mat_<FltT>& g, const size_t idx, const VecMax& init_update, VecMax& final_update) const = 0;

	FltT alpha_;
	FltT sqr_tolerance_;
	VecMax epsilon_, epsilon_inv_;
	u32 max_iter_;
};

template <typename FltT, size_t DimMax, size_t IdxBegin, size_t IdxEnd, class DataT, class GuessT, class IndicesT>
SubspaceGaussNewton<FltT, DimMax, IdxBegin, IdxEnd, DataT, GuessT, IndicesT>::SubspaceGaussNewton()
	: alpha_(cast(1.))
	, sqr_tolerance_(cast(1e-8))
	, epsilon_(VecMax::all(cast(1e-4)))
	, epsilon_inv_(VecMax::all(cast(1e4)))
	, max_iter_(100u)
{

}

template <typename FltT, size_t DimMax, size_t IdxBegin, size_t IdxEnd, class DataT, class GuessT, class IndicesT>
SubspaceGaussNewton<FltT, DimMax, IdxBegin, IdxEnd, DataT, GuessT, IndicesT>::~SubspaceGaussNewton()
{

}

template <typename FltT, size_t DimMax, size_t IdxBegin, size_t IdxEnd, class DataT, class GuessT, class IndicesT>
bool SubspaceGaussNewton<FltT, DimMax, IdxBegin, IdxEnd, DataT, GuessT, IndicesT>::compute(const DataT& data, const IndicesT &indices, GuessT& guess) const
{
	// (1) Check
	if (!checkInputValidity_(data, indices, guess))
	{
		return false;
	}

	// (2) Convert initial guess to parameter vector which is to be optimized
	VecMax param(VecMax::zeros());
	bool ret = cvtGuess2Param_(guess, indices, param);

	// (3) Prepare for the iteration
	bool condition = ret;	// Iterate at least once
	u32 iter = 0;
#ifdef SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
	FltT prev_ssd = std::numeric_limits<FltT>::max();
	FltT prev_norm_update = std::numeric_limits<FltT>::max();
#endif

	// (4) Iterate optimization
	while (condition)
	{
		ret = true;
		FltT max_norm_update = cast(0.0);
		FltT latest_ssd = cast(0.0);

		// Iterate individual optimization for each target dimension
		for (size_t i = IdxBegin; i < IdxEnd; ++i)
		{
			FltT norm_update = cast(0.0);
			ret &= computeOnce_(data, indices, i, guess, norm_update, latest_ssd);
			max_norm_update = std::max(max_norm_update, norm_update);		// Norm of update must be summed up because it is for only current subspace while SSD is for all samples.
		}

		// Check status
		if (ret)
		{
			const bool is_not_converged = (max_norm_update > sqr_tolerance_);
			const bool is_not_final_iter = (iter < max_iter_);
			condition = (is_not_converged & is_not_final_iter);
#ifdef SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
			std::cout << "[" << iter << "] ssd:    " << latest_ssd << "  <- previous: " << prev_ssd << std::endl;
			std::cout << "[" << iter << "] update: " << max_norm_update << "  <- previous: " << prev_norm_update << std::endl;
			std::cout << "[" << iter << "] is_not_converged:  " << (is_not_converged ? "T" : "F") << std::endl;
			std::cout << "[" << iter << "] is_not_final_iter: " << (is_not_final_iter ? "T" : "F") << std::endl;
			prev_ssd = latest_ssd;
			prev_norm_update = max_norm_update;
#else
			static_cast<void>(latest_ssd);
#endif
		}
		else
		{
			condition = false;
			break;
		}

		++iter;
	}

	return ret;
}

template <typename FltT, size_t DimMax, size_t IdxBegin, size_t IdxEnd, class DataT, class GuessT, class IndicesT>
bool SubspaceGaussNewton<FltT, DimMax, IdxBegin, IdxEnd, DataT, GuessT, IndicesT>::computeOnce_(const DataT &data, const IndicesT& indices, const size_t idx, GuessT &guess, FltT& norm_update, FltT& ssd) const
{
	VecMax param(VecMax::zeros());
	bool ret = cvtGuess2Param_(guess, indices, param);

	// Compute Jacobian
	const cv::Mat_<FltT> e = objFunc_(data, indices, guess);
	cv::Mat_<FltT> J(e.rows, DimMax, cast(0.));
	for (i32 i = 0; i < J.cols; ++i)
	{
		VecMax param_dash = param;
		GuessT guess_dash(guess);
		param_dash(i) += epsilon_(i);
		if (!cvtParam2Guess_(param_dash, indices, guess_dash))
		{
			ret = false;
			break;
		}
		const cv::Mat_<FltT> e_dash = objFunc_(data, indices, guess_dash);
		J.col(i) = (e_dash - e) * epsilon_inv_(i);		// Partial derivative of 'e'
#ifdef SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
		//std::cout << "J.col(" << i << ").t():" << std::endl << J.col(i).t() << std::endl;
#endif
	}

	// Solve normal equation
	VecMax update(VecMax::zeros());	// Initial delta is set zero.
	ret = solveNormalEquation_(J, e, idx, update, update);
	if (ret)
	{
		// Update param
		param = param + alpha_ * update;

		// Convert optimized parameter vector to final guess
		ret = cvtParam2Guess_(param, indices, guess);
		const cv::Mat_<FltT> e_updated = objFunc_(data, indices, guess);
		norm_update = update.dot(update);
		ssd = cv::sum(e_updated).val[0];

#ifdef SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
#if 0
		//std::cout << "(" << idx << ") param:  " << std::endl << param.t() << std::endl;
		std::cout << "(" << idx << ") update: " << std::endl << update.t() << std::endl;
		std::cout << "(" << idx << ") ssd:    " << std::endl << e_updated.t() << std::endl;
#endif
#endif
	}

	return ret;
}

template <typename FltT, size_t DimMax, size_t IdxBegin, size_t IdxEnd, class DataT, class GuessT, class IndicesT>
bool SubspaceGaussNewton<FltT, DimMax, IdxBegin, IdxEnd, DataT, GuessT, IndicesT>::solveNormalEquation_(const cv::Mat_<FltT> &J, const cv::Mat_<FltT> &e, const size_t idx, const VecMax &init_update, VecMax &final_update) const
{
	const cv::Mat_<FltT> Jt = J.t();	// Assuming the case that J is not a square matrix.
	const cv::Mat_<FltT> H = Jt * J;
	const cv::Mat_<FltT> g = Jt * e;

#ifdef SUBSPACE_GAUSS_NEWTON_CONSOLE_LOG
#if 0
	std::cout << "H:" << std::endl << H << std::endl;
	std::cout << "Jt:" << std::endl << Jt << std::endl;
	std::cout << "e:" << std::endl << e << std::endl;
	std::cout << "g.t():" << std::endl << g.t() << std::endl;
#endif
#endif

	const bool ret = solveSubspace_(H, g, idx, init_update, final_update);

	return ret;
}


#endif