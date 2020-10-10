#include "feature_visualizer.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifdef FEATURE_VISUALIZER_CONSOLE_LOG
#include <iostream>
#endif

FeatureVisualizer::FeatureVisualizer()
{
	static_cast<void>(this->clear());
}

FeatureVisualizer::~FeatureVisualizer()
{
	static_cast<void>(this->finalize());
}

bool FeatureVisualizer::clear()
{
	return true;
}

bool FeatureVisualizer::show(const Frame& frame, const KeypointVec& keypoints, const TrackVec& tracks)
{
	bool ret = true;

	cv::Mat img_vis;
	cv::cvtColor(frame.img, img_vis, cv::COLOR_GRAY2BGR);

	const size_t& num_curr_keypoints = std::count_if(keypoints.cbegin(), keypoints.cend(),
		[&frame](const Keypoint& kp) { return kp.frame_id == frame.frame_id; });
	std::printf("[Frame%lld] %zu/%zu keypoints, total %zu tracks\n", frame.frame_id, num_curr_keypoints, keypoints.size(), tracks.size());

	for (const auto& keypoint : keypoints)
	{
		if (keypoint.frame_id != frame.frame_id)
		{
			continue;
		}
		cv::Point2i pt = keypoint.coord;
		cv::circle(img_vis, pt, 2, CV_RGB(255, 255, 0));
	}
#if 1
	for (const auto& track : tracks)
	{
		if (track.keypoint_ids.empty())
		{
			std::cerr << "WARNING: Empty track is found: Track" << track.track_id << std::endl;
			continue;
		}

		bool contains_curr_frame = false;
		cv::Point2i prev_pt;

		// Assuming keypoint IDs are stored in ascending order of frame ID.
		auto keypoint_id = track.keypoint_ids.crbegin();
		constexpr i32 MAX_FLOW_LENGTH = 5;
		for (i32 i = 0; (i < MAX_FLOW_LENGTH) && (keypoint_id != track.keypoint_ids.crend()); ++i, ++keypoint_id)
		{
			auto curr_kp = std::find_if(keypoints.cbegin(), keypoints.cend(),
				[&keypoint_id](const Keypoint& kp) { return (kp.keypoint_id == *keypoint_id); });
			if (curr_kp == keypoints.cend())
			{
				continue;
			}

			const cv::Point2i curr_pt = curr_kp->coord;

			if (curr_kp->frame_id == frame.frame_id)
			{
				cv::circle(img_vis, curr_pt, 2, CV_RGB(255, 0, 0));
				contains_curr_frame = true;
			}
			else
			{
				if (contains_curr_frame)
				{
					if (curr_kp->frame_id == frame.frame_id - 1)
					{
						cv::line(img_vis, prev_pt, curr_pt, CV_RGB(255, 255, 0));
					}
					else
					{
						cv::line(img_vis, prev_pt, curr_pt, CV_RGB(128, 128, 0));
					}
				}
			}
			prev_pt = curr_pt;
		}
	}
#endif

	cv::imshow("Keypoint", img_vis);
	cv::waitKey(10);

	// Save video
	if (!output_path_.empty())
	{
		if (!writer_.isOpened())
		{
			i32 fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
			if (!writer_.open(output_path_, fourcc, 30.0, img_vis.size(), true))
			{
				std::cerr << "WARNING: Failed to open video in write mode: " << output_path_ << std::endl;
			}
		}

		if (writer_.isOpened())
		{
			writer_.write(img_vis);
		}
	}

	return ret;
}

bool FeatureVisualizer::initializeImpl_()
{
	return true;
}

bool FeatureVisualizer::finalizeImpl_()
{
	return true;
}