#include "perspective_camera.hpp"
#include "flt_util.hpp"
#include <cmath>


template <typename FltT>
PerspectiveCamera<FltT>::PerspectiveCamera()
{
	static_cast<void>(this->clear());
}

template <typename FltT>
PerspectiveCamera<FltT>::~PerspectiveCamera()
{
	static_cast<void>(this->finalize());
}

template <typename FltT>
void PerspectiveCamera<FltT>::setParameter(const PerspectiveCameraParameter& param)
{
	param_ = param;
	this->setBaseParameter_(param.base);
}

template <typename FltT>
bool PerspectiveCamera<FltT>::clear()
{
	// Do nothing
	return true;
}

template <typename FltT>
bool PerspectiveCamera<FltT>::cvtImagePt2CamRay(const FltT (&pt2d)[2], FltT (&ray3d)[3], const bool normalize) const
{
	if (!this->isInitialized())
	{
		return false;
	}
	if ((FltUtil::isZero(param_.fx)) || (FltUtil::isZero(param_.fy)))
	{
		return false;
	}

	// Multiply (fx*fy) to each element to avoid time-consuming division operator.
	bool ret = true;
	ray3d[0] = (pt2d[0] - param_.cx) * param_.fy;
	ray3d[1] = (pt2d[1] - param_.cy) * param_.fx;
	ray3d[2] = static_cast<FltT>(
		static_cast<f64>(param_.fx) * param_.fy		// To avoid digit loss due to multiplication of float
		);

	if (normalize)
	{
		const FltT norm = std::sqrt(ray3d[0] * ray3d[0] + ray3d[1] * ray3d[1] + ray3d[2] * ray3d[2]);
		if (FltUtil::isZero(norm))
		{
			ret = false;
		}
		else
		{
			const FltT norm_inv = static_cast<FltT>(1.) / norm;
			ray3d[0] *= norm_inv;
			ray3d[1] *= norm_inv;
			ray3d[2] *= norm_inv;
		}
	}

	return ret;
}

template <typename FltT>
bool PerspectiveCamera<FltT>::cvtCamRay2ImagePt(const FltT (&ray3d)[3], FltT (&pt2d)[2]) const
{
	if (!this->isInitialized())
	{
		return false;
	}

	if (FltUtil::isZero(ray3d[2]))
	{
		// In case z = 0.0
		pt2d[0] = static_cast<FltT>(param_.cx);
		pt2d[1] = static_cast<FltT>(param_.cy);
	}
	else
	{
		// In case z != 0.0
		const FltT z_inv = 1.f / ray3d[2];
		pt2d[0] = ray3d[0] * z_inv * static_cast<FltT>(param_.fx) + static_cast<FltT>(param_.cx);
		pt2d[1] = ray3d[1] * z_inv * static_cast<FltT>(param_.fy) + static_cast<FltT>(param_.cy);
	}

	return this->isInsideImage(pt2d);
}

template <typename FltT>
bool PerspectiveCamera<FltT>::initializeImpl_()
{
	// Do nothing
	return true;
}

template <typename FltT>
bool PerspectiveCamera<FltT>::finalizeImpl_()
{
	// Do nothing
	return true;
}

// Explicit instantiation
template class PerspectiveCamera<f32>;
template class PerspectiveCamera<f64>;