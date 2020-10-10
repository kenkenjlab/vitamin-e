#include "video_loader.hpp"
#include <opencv2/imgproc.hpp>


VideoLoader::VideoLoader()
	: frame_count_(0)
	, timestamp_(0.0)
	, fps_(0.0)
	, fps_inv_(std::numeric_limits<f64>::max())
	, width_(0)
	, height_(0)
{
	static_cast<void>(this->clear());
}

VideoLoader::~VideoLoader()
{
	static_cast<void>(this->finalize());
}

bool VideoLoader::clear()
{
	frame_count_ = 0u;
	timestamp_ = 0.0;
	fps_ = 0.0;
	fps_inv_ = std::numeric_limits<f64>::max();
	width_ = 0;
	height_ = 0;
	return true;
}

Frame VideoLoader::acquire()
{
	Frame frame;
	cv::Mat mat;
	cap_ >> mat;

	// Convert color
	const i32 channels = mat.channels();
	if (3 == channels)
	{
		cv::cvtColor(mat, frame.img, cv::COLOR_BGR2GRAY);
	}
	else if (4 == channels)
	{
		cv::cvtColor(mat, frame.img, cv::COLOR_BGRA2GRAY);
	}
	else if (1 == channels)
	{
		frame.img = mat;
	}
	else
	{
		// Unknown channels
		assert(false && "Unknown channel video is given.");
	}

	// Calculate frame count
	frame.frame_id = frame_count_;
	++frame_count_;

	// Calculate timestamp
	frame.timestamp = timestamp_;
	timestamp_ += fps_inv_;

	return frame;
}

bool VideoLoader::initializeImpl_()
{
	cap_.open(file_path_);
	const bool ret = cap_.isOpened();
	if (ret)
	{
		// Get FPS
		fps_ = cap_.get(cv::CAP_PROP_FPS);
		if (fps_ > 0.0)
		{
			fps_inv_ = 1.0 / fps_;
		}

		// Get image size
		width_ = static_cast<i32>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
		height_ = static_cast<i32>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

		// Get the number of frames
		num_frames_ = static_cast<FrameId>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
	}
	return ret;
}

bool VideoLoader::finalizeImpl_()
{
	cap_.release();
	return true;
}