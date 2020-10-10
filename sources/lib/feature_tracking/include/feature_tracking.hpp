#ifndef FEATURE_TRACKING_HPP
#define FEATURE_TRACKING_HPP

#include "abstract_worker.hpp"
#include "frame.hpp"
#include "keypoint.hpp"
#include "track.hpp"
#include "feature_tracking_parameter.hpp"

//#define FEATURE_TRACKING_CONSOLE_LOG

//#define FEATURE_TRACKING_USE_LKT_FOR_TRACKING
//#define FEATURE_TRACKING_USE_ORB_FOR_FEATURE

class FeatureTracking : public AbstractWorker
{
public:
	
	using Feature = cv::Point2f;
	using FeatureVec = std::vector<Feature>;
	using DMatchVec = std::vector<cv::DMatch>;
	using BoolVec = std::vector<bool>;
	using TrackIdVec = std::vector<TrackId>;

	struct SingleIntermediate
	{
		FrameId frame_id;
		Image frame_ds, frame_ds_curv, frame_curv;
		FeatureVec features_ds, features_full;
		cv::Mat descriptor;

		SingleIntermediate()
			: frame_id(INVALID_FRAME_ID)
		{}
	};

	struct Intermediate
	{
		bool is_first_time;
		SingleIntermediate prev, curr;
		DMatchVec matches;
		cv::Matx22f A_dom;
		Feature b_dom;
		FeatureVec features_curr_pred, features_curr_ref, features_merged;
		BoolVec status_ref;
		TrackIdVec track_ids;
		BoolVec nms_table;

		Intermediate()
			: is_first_time(true)
		{}
	};

	FeatureTracking();
	~FeatureTracking();

	inline void setParameter(const FeatureTrackingParameter& param, i32 width, i32 height)
	{
		param_ = param;
		width_ = width;
		height_ = height;
	}

	inline const FeatureTrackingParameter& getParameters() const { return param_; }
	
	inline const Intermediate& getIntermediates() const { return intm_; }

	bool clear() override;

	bool compute(
		const Frame& frame,
		KeypointVec& keypoints,
		TrackVec& tracks
	);

private:
	bool initializeImpl_() override;

	bool finalizeImpl_() override;


	// [Coarse Matching]

	static bool downsampleImage_(
		const cv::Mat& frame_src,
		cv::Mat& frame_dst,
		const f32& downsample_factor
	);

	static bool createCurvatureImage_(
		const cv::Mat& frame_src,
		cv::Mat& frame_dst
	);

	static bool extractFeature_(
		const cv::Mat& frame,
		FeatureVec& keypoints
	);

	static bool describeFeature_(
		const cv::Mat& frame,
		FeatureVec& keypoints,
		cv::Mat& descriptors
	);

	static bool matchFeature_(
		const cv::Mat& descriptors_prev,
		const cv::Mat& descriptors_curr,
		DMatchVec& matches
	);

	static bool estimateDominantFlow_(
		const FeatureVec& features_prev,
		const FeatureVec& features_curr,
		const DMatchVec& matches,
		cv::Matx22f& A, Feature& b
	);


	// [Fine Tracking]

	static bool predictFlow_(
		const FeatureVec& features_prev,
		FeatureVec& features_curr_est,
		const cv::Matx22f& A,
		const Feature& b
	);

	static bool findLocalExtrema_(
		const FeatureVec& features_src,
		FeatureVec& features_dst,
		BoolVec& status,
		const cv::Mat& frame_curv,
		const f32& lambda,
		const f32& sigma,
		const i32& margin
	);

	static bool refineFeature_(
		const cv::Mat& frame,
		FeatureVec& keypoints
	);


	// [Merge]

	static bool extractFlow_(
		const FeatureVec& features_prev,
		const FeatureVec& features_curr,
		const BoolVec& status_curr,
		const FrameId prev_frame_id,
		const FrameId curr_frame_id,
		TrackIdVec& track_ids,
		KeypointVec& keypoints_total,
		TrackVec& tracks
	);

	static bool mergeFeature_(
		const FeatureVec& features_new,
		const FeatureVec& features_trk,
		const i32 width,
		const i32 height,
		const u8 nms_win_size,
		BoolVec& nms_table,
		TrackIdVec& track_ids,
		FeatureVec& features_merged
	);

	static void updateNmsTable_(
		const Feature& feature,
		const i32 width,
		const i32 height,
		const u8 nms_win_size,
		BoolVec& nms_table
	);

	FeatureTrackingParameter param_;
	i32 width_;
	i32 height_;

	Intermediate intm_;
};

#endif