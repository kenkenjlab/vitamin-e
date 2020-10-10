#ifndef POINT_OPTIMIZATION_PARAMETER_HPP
#define POINT_OPTIMIZATION_PARAMETER_HPP

#include "definition.hpp"
#include "abstract_parameter.hpp"

struct PointOptimizationParameter : public AbstractParameter
{
	u32 min_votes;
	OptFltT min_err;

	PointOptimizationParameter()
		: min_votes(2u)
		, min_err(0.5)
	{}

	bool isValid() const override;
};

#endif