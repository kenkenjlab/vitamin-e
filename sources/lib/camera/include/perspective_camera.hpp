#ifndef PERSPECTIVE_CAMERA_HPP
#define PERSPECTIVE_CAMERA_HPP

#include "camera.hpp"
#include "perspective_camera_parameter.hpp"

template <typename FltT>
class PerspectiveCamera : public Camera<FltT>
{
public:
	PerspectiveCamera();
	~PerspectiveCamera();

	bool clear() override;

	void setParameter(const PerspectiveCameraParameter& param);
	inline const PerspectiveCameraParameter& getParameter() const { return param_; }

	bool cvtImagePt2CamRay(const FltT (&pt2d)[2], FltT (&ray3d)[3], const bool normalize) const override;
	bool cvtCamRay2ImagePt(const FltT (&ray3d)[3], FltT (&pt2d)[2]) const override;

protected:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	PerspectiveCameraParameter param_;
};

#endif