#include "camera.hpp"

template <typename FltT>
Camera<FltT>::Camera()
{}

template <typename FltT>
Camera<FltT>::~Camera()
{}

template <typename FltT>
bool Camera<FltT>::isInsideImage(const FltT(&pt2d)[2]) const
{
	return (pt2d[0] >= 0.0f) && (pt2d[1] >= 0.0f) && (pt2d[0] < param_.width) && (pt2d[1] < param_.height);
}

// Explicit instantiation
template class Camera<f32>;
template class Camera<f64>;