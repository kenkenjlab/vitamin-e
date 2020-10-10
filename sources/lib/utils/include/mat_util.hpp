#ifndef MAT_UTIL_HPP
#define MAT_UTIL_HPP

#include <opencv2/core.hpp>

template <typename FltT>
class MatUtil
{
public:
	static void genRotationMatrix(const cv::Matx<FltT, 3, 1>& radians, cv::Matx<FltT, 3, 3>& matrix);
	static void genAngle(const cv::Matx<FltT, 3, 3>& matrix, cv::Matx<FltT, 3, 1>& radians);
};

#endif