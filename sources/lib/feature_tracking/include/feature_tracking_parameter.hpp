#ifndef FEATURE_TRACKING_PARAMETER_HPP
#define FEATURE_TRACKING_PARAMETER_HPP

#include "definition.hpp"
#include "abstract_parameter.hpp"

struct FeatureTrackingParameter : public AbstractParameter
{
	f32 downsample_factor;
	f32 refinement_lambda;
	f32 refinement_sigma;
	u8 nms_win_size;

	FeatureTrackingParameter();

	bool isValid() const override;
};

#endif