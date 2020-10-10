#ifndef WORLD_VISUALIZER_HPP
#define WORLD_VISUALIZER_HPP

#include "abstract_worker.hpp"
#include <string>
#include "frame.hpp"
#include "point.hpp"
#include "camera_pose.hpp"
#include "perspective_camera_parameter.hpp"
#include <opencv2/viz.hpp>

#define WORLD_VISUALIZER_CONSOLE_LOG

class WorldVisualizer : public AbstractWorker
{
public:

	explicit WorldVisualizer(const std::string& name);
	~WorldVisualizer();

	bool clear() override;

	void addAxis(const Mat44& pose, const std::string& id, const f64 scale);

	void addPoints(const PointVec& points, const std::string& id, const cv::viz::Color& color);

	void addCameras(const CameraPoseVec& camera_poses, const PerspectiveCameraParameter& cam_param, const std::string& id, const f64 scale, const cv::viz::Color& color);

	void addCameras(const CameraPoseVec& camera_poses, const std::vector<Frame>& frames,const PerspectiveCameraParameter& cam_param, const std::string& id, const f64 scale, const cv::viz::Color& color);

	void addPointCorrespondences(const PointVec& points1, const PointVec& points2, const std::string& id, const cv::viz::Color& color);

	void addCameraPoseCorrespondences(const CameraPoseVec& camera_poses1, const CameraPoseVec& camera_poses2, const std::string& id, const cv::viz::Color& color);

	inline void spin() { window_.spin(); }

	inline void spinOnce(const i32 time = 1, const bool force_redraw =  false) { window_.spinOnce(time, force_redraw); }

private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	template <typename NumT>
	static std::string genIdStr_(const std::string& id_str, NumT number);

	static cv::Mat cvtPoints2Mat_(const PointVec& points);

	cv::viz::Viz3d window_;
};

#endif