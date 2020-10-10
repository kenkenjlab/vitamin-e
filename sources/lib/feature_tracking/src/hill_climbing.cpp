#include "hill_climbing.hpp"

template <typename FrameValT>
HillClimbing<FrameValT>::HillClimbing()
	: lambda_(1.f)
	, sqr_sigma_(0.01f)
	, margin_(0)
{

}

template <typename FrameValT>
HillClimbing<FrameValT>::~HillClimbing()
{

}

template <typename FrameValT>
bool HillClimbing<FrameValT>::setFrame(const cv::Mat &frame)
{
	// Check whether T and value type of frame are same or not.
	const bool ret = (sizeof(FrameValT) == frame.elemSize1());
	if (ret)
	{
		frame_ = frame;
	}
	return ret;
}

template <typename FrameValT>
bool HillClimbing<FrameValT>::compute(const cv::Point &point_src, cv::Point &point_dst) const
{
	if (frame_.empty())
	{
		return false;
	}
	if (isOutOfFrame_(point_src))
	{
		return false;
	}

	cv::Point point_best = point_src;
	f32 score_best = std::numeric_limits<f32>::lowest();
	bool reject = false;
	while (!reject)
	{
		cv::Point point_next = point_best;
		f32 score_next = std::numeric_limits<f32>::lowest();

		// Search 8 neighbors
		for (i32 y = -1; (y <= 1) && (!reject); ++y)
		{
			for (i32 x = -1; (x <= 1) && (!reject); ++x)
			{
				if ((x == 0) && (y == 0))
				{
					continue;
				}

				// Generate current neighbor to check
				const cv::Point point_curr = point_next + cv::Point(x, y);

				// Terminate if the neighbor is out of image
				if (isOutOfFrame_(point_curr))
				{
					reject = true;
					break;
				}

				const f32 score_curr = evalFunc_(point_src, point_curr);
				if (score_next <= score_curr)
				{
					score_next = score_curr;
					point_next = point_curr;
				}
			}
		}

		// Overwrite current best as global best
		if (score_next <= score_best)
		{
			break;
		}
		point_best = point_next;
		score_best = score_next;
	}

	// Pick up the best
	point_dst = point_best;

	return !reject;
}

template <typename FrameValT>
f32 HillClimbing<FrameValT>::evalFunc_(const cv::Point &point_init, const cv::Point &point_est) const
{
	const FrameValT val = frame_.at<FrameValT>(point_est);
	const cv::Point point_diff = point_est - point_init;
	const f32 sqr_norm = static_cast<f32>(point_diff.dot(point_diff));
	const f32 reg = sqr_sigma_ / (sqr_sigma_ + sqr_norm);
	const f32 eval = static_cast<f32>(val) + lambda_ * reg;
	return eval;
}

template <typename FrameValT>
bool HillClimbing<FrameValT>::isOutOfFrame_(const cv::Point &pt) const
{
	return ((pt.x < margin_) || (pt.y < margin_) || (pt.x >= frame_.cols - margin_) || (pt.y >= frame_.rows - margin_));
}

// Explicit instantiation
template class HillClimbing<i8>;
template class HillClimbing<i16>;
template class HillClimbing<i32>;
template class HillClimbing<f32>;
template class HillClimbing<f64>;