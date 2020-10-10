#include "flt_util.hpp"
#include "definition.hpp"
#include <limits>

template <typename FltT>
bool FltUtil::isZero(const FltT &val)
{
	return (val <= std::numeric_limits<FltT>::epsilon()) && (val >= -std::numeric_limits<FltT>::epsilon());
}

template bool FltUtil::isZero<f32>(const f32 &val);
template bool FltUtil::isZero<f64>(const f64 &val);