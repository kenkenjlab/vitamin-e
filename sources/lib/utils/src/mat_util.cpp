#include "mat_util.hpp"
#include <cmath>

template <typename FltT>
void MatUtil<FltT>::genRotationMatrix(const cv::Matx<FltT, 3, 1>& radians, cv::Matx<FltT, 3, 3>& matrix)
{
	using Mat33 = cv::Matx<FltT, 3, 3>;

	Mat33 Rx(Mat33::eye()), Ry(Mat33::eye()), Rz(Mat33::eye());
	const FltT c0 = std::cos(radians(0));
	const FltT s0 = std::sin(radians(0));
	const FltT c1 = std::cos(radians(1));
	const FltT s1 = std::sin(radians(1));
	const FltT c2 = std::cos(radians(2));
	const FltT s2 = std::sin(radians(2));
	Rx(1, 1) = c0;
	Rx(1, 2) = -s0;
	Rx(2, 1) = s0;
	Rx(2, 2) = c0;
	Ry(0, 0) = c1;
	Ry(0, 2) = s1;
	Ry(2, 0) = -s1;
	Ry(2, 2) = c1;
	Rz(0, 0) = c2;
	Rz(0, 1) = -s2;
	Rz(1, 0) = s2;
	Rz(1, 1) = c2;
	matrix = Rz * Ry * Rx;
}

template <typename FltT>
void MatUtil<FltT>::genAngle(const cv::Matx<FltT, 3, 3>& matrix, cv::Matx<FltT, 3, 1>& radians)
{
	// Note: roll and pitch (radians(0), (1)) are assumed to be between -pi/2 and pi/2 while yaw (radians(2)) is assumed to be between -pi and pi
	radians(1) = std::asin(-matrix(2, 0));
	const FltT c1 = std::cos(radians(1));
	radians(0) = std::asin(matrix(2, 1) / c1);
	radians(2) = std::asin(matrix(1, 0) / c1);
}


// Explicit instantiations
template class MatUtil<float>;
template class MatUtil<double>;