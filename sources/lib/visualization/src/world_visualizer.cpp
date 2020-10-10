#include "world_visualizer.hpp"
#include <sstream>

#ifdef WORLD_VISUALIZER_CONSOLE_LOG
#include <iostream>
#endif

WorldVisualizer::WorldVisualizer(const std::string& name)
	: window_(name)
{
	static_cast<void>(this->clear());
}

WorldVisualizer::~WorldVisualizer()
{
	static_cast<void>(this->finalize());
}

bool WorldVisualizer::clear()
{
	window_.removeAllWidgets();
	window_.close();
	return true;
}

void WorldVisualizer::addAxis(const Mat44& pose, const std::string& id, const f64 scale)
{
	window_.showWidget(id, cv::viz::WCoordinateSystem(scale), cv::Affine3f(pose));
}

void WorldVisualizer::addPoints(const PointVec& points, const std::string& id, const cv::viz::Color& color)
{
	if (points.empty())
	{
		return;
	}

	// Generate point cloud mat
	cv::Mat cloud = cvtPoints2Mat_(points);

	// Generate widget
	cv::viz::WCloud wcloud(cloud, color);
	wcloud.setRenderingProperty(cv::viz::POINT_SIZE, 4.0);

	// Add to window
	window_.showWidget(id, wcloud);
}

void WorldVisualizer::addCameras(const CameraPoseVec& camera_poses, const PerspectiveCameraParameter& cam_param, const std::string& id, const f64 scale, const cv::viz::Color& color)
{
	std::vector<Frame> frames;
	addCameras(camera_poses, frames, cam_param, id, scale, color);
}

void WorldVisualizer::addCameras(const CameraPoseVec& camera_poses, const std::vector<Frame>& frames, const PerspectiveCameraParameter& cam_param, const std::string& id, const f64 scale, const cv::viz::Color& color)
{
	if(camera_poses.empty())
	{
		return;
	}

	cv::Matx33d K;
	K << static_cast<f64>(cam_param.fx), 0.0, static_cast<f64>(cam_param.cx),
		0.0, static_cast<f64>(cam_param.fy), static_cast<f64>(cam_param.cy),
		0.0, 0.0, 1.0;
	for (const auto& cam_pose : camera_poses)
	{
		// Generate ID
		std::string curr_id(genIdStr_(id, cam_pose.frame_id));

		// Get corresponding frame
		auto frame = std::find_if(frames.cbegin(), frames.cend(),
			[&cam_pose](const Frame& f) { return (f.frame_id == cam_pose.frame_id); });

		// Generate widget
		cv::viz::WCameraPosition wcam;
		if (frame == frames.cend())
		{
			wcam = cv::viz::WCameraPosition(K, scale, color);
		}
		else
		{
			wcam = cv::viz::WCameraPosition(K, frame->img, scale, color);
		}
		wcam.setRenderingProperty(cv::viz::OPACITY, 0.5);
		cv::Affine3f aff(cam_pose.getTransformationMatrix());

		// Add to window
		window_.showWidget(curr_id, wcam, aff);
	}
}

void WorldVisualizer::addPointCorrespondences(const PointVec& points1, const PointVec& points2, const std::string& id, const cv::viz::Color& color)
{
	for (const auto& pt1 : points1)
	{
		// Find correspondence
		const TrackId track_id = pt1.track_id;
		auto pt2_it = std::find_if(points2.cbegin(), points2.cend(),
			[&track_id](const Point& pt) { return (pt.track_id == track_id); });
		if ((pt2_it == points2.cend())
			|| (track_id < 0))
		{
			continue;
		}
		auto pt2 = *pt2_it;

		// Generate ID
		std::string curr_id(genIdStr_(id, track_id));
		
		// Generate widget
		cv::viz::WLine wline(pt1.getCvPoint3d(), pt2.getCvPoint3d(), color);

		// Add to window
		window_.showWidget(curr_id, wline);
	}
}

void WorldVisualizer::addCameraPoseCorrespondences(const CameraPoseVec& camera_poses1, const CameraPoseVec& camera_poses2, const std::string& id, const cv::viz::Color& color)
{
	for (const auto& cam_pose1 : camera_poses1)
	{
		// Find correspondence
		const FrameId frame_id = cam_pose1.frame_id;
		auto cam_pose2_it = std::find_if(camera_poses2.cbegin(), camera_poses2.cend(),
			[&frame_id](const CameraPose& cp) { return (cp.frame_id == frame_id); });
		if ((cam_pose2_it == camera_poses2.cend())
			|| (frame_id < 0))
		{
			continue;
		}
		auto cam_pose2 = *cam_pose2_it;

		// Generate ID
		std::string curr_id(genIdStr_(id, frame_id));

		// Generate widget
		cv::viz::WLine wline(cam_pose1.getCvPoint3d(), cam_pose2.getCvPoint3d(), color);

		// Add to window
		window_.showWidget(curr_id, wline);
	}
}

bool WorldVisualizer::initializeImpl_()
{
	return true;
}

bool WorldVisualizer::finalizeImpl_()
{
	return true;
}

template <typename NumT>
std::string WorldVisualizer::genIdStr_(const std::string& id_str, NumT number)
{
	std::stringstream ss;
	ss << id_str << "_" << number;
	return ss.str();
}

cv::Mat WorldVisualizer::cvtPoints2Mat_(const PointVec& points)
{
	i32 num_pt = static_cast<i32>(points.size());
	cv::Mat cloud_mat(num_pt, 1, CV_32FC3);
	for (i32 i = 0; i < num_pt; ++i)
	{
		cloud_mat.at<cv::Vec3f>(i, 0) = cv::Mat(points[i].coord);
	}
	return cloud_mat;
}

// Explicit Instantiations
template std::string WorldVisualizer::genIdStr_<i8>(const std::string& id_str, i8 number);
template std::string WorldVisualizer::genIdStr_<i16>(const std::string& id_str, i16 number);
template std::string WorldVisualizer::genIdStr_<i32>(const std::string& id_str, i32 number);
template std::string WorldVisualizer::genIdStr_<i64>(const std::string& id_str, i64 number);
template std::string WorldVisualizer::genIdStr_<u8>(const std::string& id_str, u8 number);
template std::string WorldVisualizer::genIdStr_<u16>(const std::string& id_str, u16 number);
template std::string WorldVisualizer::genIdStr_<u32>(const std::string& id_str, u32 number);
template std::string WorldVisualizer::genIdStr_<u64>(const std::string& id_str, u64 number);
template std::string WorldVisualizer::genIdStr_<f32>(const std::string& id_str, f32 number);
template std::string WorldVisualizer::genIdStr_<f64>(const std::string& id_str, f64 number);