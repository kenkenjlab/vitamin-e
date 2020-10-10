#include "calib_loader.hpp"
#include <opencv2/core.hpp>

bool CalibLoader::load(const std::string file_path, PerspectiveCameraParameter&param)
{
	cv::FileStorage fs(file_path, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		return false;
	}

	param.base.width = static_cast<u32>(static_cast<int>(fs["width"]));
	param.base.height = static_cast<u32>(static_cast<int>(fs["height"]));
	param.fx = static_cast<f32>(fs["fx"]);
	param.fy = static_cast<f32>(fs["fy"]);
	param.cx = static_cast<f32>(fs["cx"]);
	param.cy = static_cast<f32>(fs["cy"]);

	param.base.setInitialized();
	param.setInitialized();
	return true;
}