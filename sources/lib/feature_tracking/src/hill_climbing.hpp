#ifndef HILL_CLIMBING_HPP
#define HILL_CLIMBING_HPP

#include "definition.hpp"
#include <opencv2/core.hpp>

template <typename FrameValT>
class HillClimbing
{
public:
	HillClimbing();
	~HillClimbing();

	bool setFrame(const cv::Mat &frame);
	inline void setLambda(const f32 &lambda) { lambda_ = lambda; }
	inline void setSigma(const f32 &sigma) { sqr_sigma_ = sigma * sigma; }
	inline void setMargin(const i32 margin) { margin_ = margin; }

	bool compute(const cv::Point &point_src, cv::Point &point_dst) const;

private:
	f32 evalFunc_(const cv::Point &point_init, const cv::Point &point_est) const;
	bool isOutOfFrame_(const cv::Point &pt) const;
	static inline i32 cast_(const f32 &val) { return static_cast<i32>(std::roundf(val)); }

	cv::Mat frame_;
	f32 lambda_;
	f32 sqr_sigma_;
	i32 margin_;
};

#endif
