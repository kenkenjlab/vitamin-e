#ifndef VIDEO_LOADERR_HPP
#define VIDEO_LOADERR_HPP

#include <opencv2/highgui.hpp>
#include "frame.hpp"
#include "abstract_worker.hpp"

class VideoLoader : public AbstractWorker
{
public:
	VideoLoader();
	~VideoLoader();

	inline void setFilePath(const std::string& file_path) { file_path_ = file_path; }
	inline const std::string& getFilePath() const { return file_path_; }

	bool clear() override;

	Frame acquire();

	inline FrameId getCurrentFrameCount() const { return frame_count_; }
	inline f64 getCurrentTimestamp() const { return timestamp_; }
	inline f64 getFps() const { return fps_; }
	inline f64 getInvFps() const { return fps_inv_; }
	inline i32 getWidth() const { return width_; }
	inline i32 getHeight() const { return height_; }
	inline FrameId getNumOfFrames() const { return num_frames_; }

private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;

	std::string file_path_;
	cv::VideoCapture cap_;
	FrameId frame_count_;
	Timestamp timestamp_;

	// Video information
	f64 fps_;
	f64 fps_inv_;
	i32 width_;
	i32 height_;
	FrameId num_frames_;
};

#endif