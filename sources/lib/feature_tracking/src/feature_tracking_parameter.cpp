#include "feature_tracking_parameter.hpp"

FeatureTrackingParameter::FeatureTrackingParameter()
	: downsample_factor(0.25f)
	, refinement_lambda(1.0f)	//(255.f * 255.f * 255.f * 0.001f)
	, refinement_sigma(1.f)
	, nms_win_size(21u)
{}

bool FeatureTrackingParameter::isValid() const
{
	bool ret = isInitialized();

	ret &= ((downsample_factor > 0.f) && (downsample_factor < 1.f));

	return ret;
}