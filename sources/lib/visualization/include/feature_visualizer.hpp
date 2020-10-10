#ifndef FEATURE_VISUALIZER_HPP
#define FEATURE_VISUALIZER_HPP

#include "abstract_worker.hpp"
#include "frame.hpp"
#include "keypoint.hpp"
#include "track.hpp"
#include <opencv2/highgui.hpp>

//#define FEATURE_VISUALIZER_CONSOLE_LOG

class FeatureVisualizer : public AbstractWorker
{
public:

	FeatureVisualizer();
	~FeatureVisualizer();

	inline void setOutputPath(const std::string& file_path) { output_path_ = file_path; }

	bool clear() override;

	bool show(const Frame& frame, const KeypointVec& keypoints, const TrackVec& tracks);

private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	std::string output_path_;
	cv::VideoWriter writer_;
};

#endif