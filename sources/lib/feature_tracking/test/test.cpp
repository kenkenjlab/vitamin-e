#include "video_loader.hpp"
#include "feature_tracking.hpp"
#include "feature_visualizer.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv)
{
	// Check argument
	if (argc < 2)
	{
		std::cerr << "ERROR: Input video is not given. Specify path to video." << std::endl;
		return 1;
	}
	const std::string video_path(argv[1]);

	// Open video
	VideoLoader video_loader;
	video_loader.setFilePath(video_path);
	if (!video_loader.initialize())
	{
		std::cerr << "ERROR: Failed to open video: " << video_path << std::endl;
		return 2;
	}

	// Prepare tracker
	FeatureTracking feat_track;
	FeatureTrackingParameter param;
	param.setInitialized();
	feat_track.setParameter(param, video_loader.getWidth(), video_loader.getHeight());
	feat_track.initialize();

	// Prepare visualizer
	FeatureVisualizer feat_vis;
	feat_vis.setOutputPath("vis.avi");
	feat_vis.initialize();

	// Track feature
	KeypointVec keypoints;
	TrackVec tracks;
	while (1)
	{
		Frame frame = video_loader.acquire();
		if (frame.empty())
		{
			break;
		}

		const bool ret = feat_track.compute(frame, keypoints, tracks);
		if (!ret)
		{
			std::cerr << "ERROR: Failed to track features" << std::endl;
			break;
		}

		feat_vis.show(frame, keypoints, tracks);
	}

	return 0;
}
