#include "test_dataset.hpp"
#include <sstream>

std::string TestDataset::str(const i8 verbose_keypoints, const i8 verbose_tracks, const bool verbose_cameras, const bool verbose_points, const std::string& prefix) const
{
	std::stringstream ss;
	ss << prefix << "Name: " << name;
	if ((verbose_keypoints > 0) || (verbose_tracks > 0))
	{
		FeatureBuffer feat_buff = getFeatureBuffer();
		ss << std::endl << prefix << feat_buff.str(verbose_keypoints, verbose_tracks, "  ");
	}
	if (verbose_cameras || verbose_points)
	{
		WorldBuffer world_buff = getWorldBuffer();
		ss << std::endl << prefix << world_buff.str(verbose_cameras, verbose_points, "  ");
	}
	return ss.str();
}