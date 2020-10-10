#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "definition.hpp"
#include "abstract_worker.hpp"
#include "camera_parameter.hpp"

template <typename FltT>
class Camera : public AbstractWorker
{
public:
	Camera();
	~Camera();

	virtual bool cvtImagePt2CamRay(const FltT (&pt2d)[2], FltT (&ray3d)[3], const bool normalize = false) const = 0;
	virtual bool cvtCamRay2ImagePt(const FltT (&ray3d)[3], FltT (&pt2d)[2]) const = 0;
	bool isInsideImage(const FltT(&pt2d)[2]) const;

protected:
	inline void setBaseParameter_(const CameraParameter& param) { param_ = param; }

private:
	CameraParameter param_;
};

#endif