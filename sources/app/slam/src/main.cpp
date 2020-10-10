#include <iostream>
#include <string>
#include "vitamin_e.hpp"
#include "video_loader.hpp"
#include "calib_loader.hpp"
#include "feature_visualizer.hpp"
#include "world_visualizer.hpp"

#define PROJECT_COPYRIGHT ("Copyright 2020 kenken, jLab.")

bool parseArguments_(
	const int argc,
	const char* const argv[],
	std::string& video_path,
	VitaminEParameter& param
);

int main(int argc, char** argv)
{
	// Parse arguments
	std::string video_path;
	VitaminEParameter vit_param;
	if (!parseArguments_(argc, argv, video_path, vit_param))
	{
		return 1;
	}

	// Open video
	VideoLoader video_loader;
	video_loader.setFilePath(video_path);
	if (!video_loader.initialize())
	{
		std::cerr << "ERROR: Failed to open video: " << video_path << std::endl;
		return 2;
	}

	// Initialize Vitamin-E
	VitaminE vit;
	vit.setParameter(vit_param, video_loader.getWidth(), video_loader.getHeight());
	if (!vit.initialize())
	{
		std::cerr << "ERROR: Failed to initialize Vitamin-E." << std::endl;
		return 3;
	}
	FeatureVisualizer feat_vis;
	if (!feat_vis.initialize())
	{
		std::cerr << "ERROR: Failed to initialize Vitamin-E Visualizer." << std::endl;
		return 4;
	}
	WorldVisualizer world_vis("World");
	if (!world_vis.initialize())
	{
		std::cerr << "ERROR: Failed to initialize Vitamin-E Visualizer." << std::endl;
		return 5;
	}

	// Iterate processing
	while (1)
	{
		// Acquire frame
		Frame frame = video_loader.acquire();
		if (frame.empty())
		{
			std::cout << "End of video." << std::endl;
			break;
		}

		// Process
		vit.processOnce(frame);

		// Visualize feature
		const KeypointVec& keypoints = vit.getKeypointBuffer();
		const TrackVec& tracks = vit.getTrackBuffer();
		feat_vis.show(frame, keypoints, tracks);

		// Visualize world
		const CameraPoseVec& camera_poses = vit.getCameraPoseBuffer();
		const PointVec& points = vit.getPointBuffer();
		world_vis.addAxis(Mat44::eye(), "origin", 1.0);
		world_vis.addCameras(camera_poses, vit_param.cam, "cam", 1.0, cv::viz::Color::blue());
		world_vis.addPoints(points, "points", cv::viz::Color::white());
		world_vis.spinOnce();
	}

	// Finalize Vitamin-E
	if (!vit.finalize())
	{
		std::cerr << "ERROR: Failed to finalize Vitamin-E." << std::endl;
	}
	if (!feat_vis.finalize())
	{
		std::cerr << "ERROR: Failed to finalize Feature Visualizer." << std::endl;
	}
	if (!world_vis.finalize())
	{
		std::cerr << "ERROR: Failed to finalize World Visualizer." << std::endl;
	}

	// Close video
	if (!video_loader.finalize())
	{
		std::cerr << "ERROR: Failed to close video." << std::endl;
	}

	return 0;
}

bool parseArguments_(
	const int argc,
	const char* const argv[],
	std::string& video_path,
	VitaminEParameter& param
)
{
	bool ret_arg = true;
	const cv::String keys =
		"{help h ?||Show help}"
		"{video||Video file path (Required)}"
		"{calib||Calib file path (Required)}"
		"{param||Parameter file path (Required)}";
	const cv::String about = PROJECT_COPYRIGHT;

	cv::CommandLineParser parser(argc, argv, keys);
	parser.about(about);

	if (parser.has("help"))
	{
		parser.printMessage();
		ret_arg = false;
	}
	else
	{
		if (parser.has("video"))
		{
			video_path = parser.get<std::string>("video");
		}
		else
		{
			std::cerr << "ERROR: Specify video file path" << std::endl;
			ret_arg = false;
		}

		if (parser.has("calib"))
		{
			// Open calibration setting
			const std::string calib_path = parser.get<std::string>("calib");
			if (!CalibLoader::load(calib_path, param.cam))
			{
				std::cerr << "ERROR: Failed to load calib path: " << calib_path << std::endl;
				ret_arg = false;
			}
		}
		else
		{
			std::cerr << "ERROR: Specify calibration file path" << std::endl;
			ret_arg = false;
		}

		if (parser.has("param"))
		{
			// TODO: IMPLEMENT ME

			param.feat_track.setInitialized();
			param.cam_pose_init_est.setInitialized();
			param.cam_pose_opt.setInitialized();
			param.point_opt.setInitialized();
		}
		else
		{
			std::cerr << "ERROR: Specify parameter file path" << std::endl;
			ret_arg = false;
		}
	}

	return ret_arg;
}