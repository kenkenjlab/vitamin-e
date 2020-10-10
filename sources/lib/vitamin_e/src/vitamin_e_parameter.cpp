#include "vitamin_e_parameter.hpp"

bool VitaminEParameter::isValid() const
{
	bool ret = isInitialized();
	ret &= feat_track.isValid();
	ret &= cam_pose_init_est.isValid();
	ret &= cam_pose_opt.isValid();
	ret &= point_opt.isValid();
	return ret;
}