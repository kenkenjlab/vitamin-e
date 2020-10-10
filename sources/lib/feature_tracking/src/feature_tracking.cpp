#include "feature_tracking.hpp"
#include "hill_climbing.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#else
#include <opencv2/features2d.hpp>
#endif

#ifdef FEATURE_TRACKING_CONSOLE_LOG
#include <iostream>
#endif

#ifdef FEATURE_TRACKING_USE_LKT_FOR_TRACKING
#include <opencv2/video/tracking.hpp>
#endif

FeatureTracking::FeatureTracking()
	: width_(0)
	, height_(0)
{
	static_cast<void>(this->clear());
}

FeatureTracking::~FeatureTracking()
{
	static_cast<void>(this->finalize());
}

bool FeatureTracking::clear()
{
	return true;
}

bool FeatureTracking::compute(
	const Frame& frame,
	KeypointVec& keypoints,
	TrackVec& tracks
)
{
	// Check
	if ((!isInitialized()) || (1 != frame.img.channels()))
	{
		return false;
	}

	bool ret = true;
	SingleIntermediate& sintm_curr = intm_.curr;
	SingleIntermediate& sintm_prev = intm_.prev;

	// (1) Store previous data before processing
	sintm_prev = sintm_curr;
	sintm_curr = SingleIntermediate();
	sintm_curr.frame_id = frame.frame_id;

#ifdef FEATURE_TRACKING_USE_LKT_FOR_TRACKING
	// Temporary used for original image buffer
	sintm_curr.frame_curv = frame.img;

	// Extract keypoints
	cv::Ptr<cv::Feature2D> detector = cv::GFTTDetector::create(frame.img.cols * frame.img.rows / 100);
	std::vector<cv::KeyPoint> keypoints_local;
	detector->detect(frame.img, keypoints_local);
	cv::KeyPoint::convert(keypoints_local, sintm_curr.features_full);

	if (!intm_.is_first_time)
	{
		// Track keypoints
		std::vector<u8> status;
		std::vector<f32> err;
		cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
		FeatureVec tracked_keypoints;
		cv::calcOpticalFlowPyrLK(
			sintm_prev.frame_curv,
			sintm_curr.frame_curv,
			intm_.features_merged,
			intm_.features_curr_ref,
			status,
			err,
			cv::Size(21, 21),
			3,
			criteria
		);
		intm_.status_ref.resize(status.size());
		for (size_t i = 0; i < status.size(); ++i)
		{
			intm_.status_ref[i] = (status[i] != 0u) ? true : false;
		}

#else	// End of #ifdef FEATURE_TRACKING_USE_LKT_FOR_TRACKING
	// (2) [Coarse Matching] Extract feature points from current downsampled image and then describe them
	ret &= downsampleImage_(
		frame.img,
		sintm_curr.frame_ds,
		param_.downsample_factor
	);
	ret &= createCurvatureImage_(
		sintm_curr.frame_ds,
		sintm_curr.frame_ds_curv
	);
	ret &= extractFeature_(
#ifdef FEATURE_TRACKING_USE_ORB_FOR_FEATURE
		sintm_curr.frame_ds,
#else
		sintm_curr.frame_ds_curv,
#endif
		sintm_curr.features_ds
	);
	ret &= describeFeature_(
		sintm_curr.frame_ds,
		sintm_curr.features_ds,
		sintm_curr.descriptor
	);
	ret &= createCurvatureImage_(
		frame.img,
		sintm_curr.frame_curv
	);
	ret &= extractFeature_(
#ifdef FEATURE_TRACKING_USE_ORB_FOR_FEATURE
		frame.img,
#else
		sintm_curr.frame_curv,
#endif
		sintm_curr.features_full
	);
	ret &= refineFeature_(
		frame.img,
		sintm_curr.features_full
	);

	// (3) Check if first time or not
	if (!intm_.is_first_time)
	{	// Not first time -> Go on
		// (a) [Coarse Matching] Estimate dominant flow
		ret &= matchFeature_(
			sintm_prev.descriptor,
			sintm_curr.descriptor,
			intm_.matches
		);
		ret &= estimateDominantFlow_(
			sintm_prev.features_ds,
			sintm_curr.features_ds,
			intm_.matches,
			intm_.A_dom,
			intm_.b_dom
		);

		// (b) [Fine Tracking] Extract feature points from previous original image, then refine
		ret &= predictFlow_(
			intm_.features_merged,
			intm_.features_curr_pred,
			intm_.A_dom,
			intm_.b_dom
		);
		ret &= findLocalExtrema_(
			intm_.features_curr_pred,
			intm_.features_curr_ref,
			intm_.status_ref,
			sintm_curr.frame_curv,
			param_.refinement_lambda,
			param_.refinement_sigma,
			param_.nms_win_size >> 1
		);
#endif		// End of #ifdef FEATURE_TRACKING_USE_LKT_FOR_TRACKING
		ret &= extractFlow_(
			intm_.features_merged,
			intm_.features_curr_ref,
			intm_.status_ref,
			sintm_prev.frame_id,
			sintm_curr.frame_id,
			intm_.track_ids,
			keypoints,
			tracks
		);
		ret &= mergeFeature_(
			sintm_curr.features_full,
			intm_.features_curr_ref,
			width_,
			height_,
			param_.nms_win_size,
			intm_.nms_table,
			intm_.track_ids,
			intm_.features_merged
		);
	}
	else
	{	// First time -> Just copy features
		intm_.features_merged = sintm_curr.features_full;
		intm_.track_ids.resize(intm_.features_merged.size(), INVALID_TRACK_ID);
		intm_.is_first_time = false;
	}

	return ret;
}

bool FeatureTracking::initializeImpl_()
{
	// Allocate table for non-maxima supression
	intm_.nms_table.resize(static_cast<size_t>(width_) * height_, false);

	// Check if parameter is valid
	return param_.isValid();
}

bool FeatureTracking::finalizeImpl_()
{
	return true;
}

bool FeatureTracking::downsampleImage_(
	const cv::Mat& frame_src,
	cv::Mat& frame_dst,
	const f32& downsample_factor
)
{
	cv::resize(frame_src, frame_dst, frame_dst.size(), downsample_factor, downsample_factor,
#if (CV_VERSION_MAJOR > 3) || ((CV_VERSION_MAJOR == 3) && (CV_VERSION_MINOR >= 4))
		cv::INTER_LINEAR_EXACT
#else
		cv::INTER_LINEAR
#endif
	);
	return true;
}

bool FeatureTracking::createCurvatureImage_(
	const cv::Mat& frame_src,
	cv::Mat& frame_dst
)
{
	cv::Mat frame_x, frame_y, frame_xx, frame_xy, frame_yy;
	constexpr i32 ddpeth = CV_32F;
	cv::Sobel(frame_src, frame_x, ddpeth, 1, 0, 5);
	cv::Sobel(frame_src, frame_y, ddpeth, 0, 1, 5);
	cv::Sobel(frame_src, frame_xx, ddpeth, 2, 0, 5);
	cv::Sobel(frame_src, frame_xy, ddpeth, 1, 1, 5);
	cv::Sobel(frame_src, frame_yy, ddpeth, 0, 2, 5);
	frame_dst = frame_y.mul(frame_y).mul(frame_xx) - 2.f * frame_x.mul(frame_y).mul(frame_xy) + frame_x.mul(frame_x).mul(frame_yy);
	frame_dst = cv::abs(frame_dst);
	return true;
}

bool FeatureTracking::extractFeature_(
	const cv::Mat& frame,
	FeatureVec& keypoints
)
{
#ifdef FEATURE_TRACKING_USE_ORB_FOR_FEATURE
	std::vector<cv::KeyPoint> points;
	cv::Ptr<cv::ORB> det = cv::ORB::create(frame.cols * frame.rows / 100);
	det->detect(frame, points);
	cv::KeyPoint::convert(points, keypoints);
#else
	cv::Mat frame_peak, frame_flat;
	const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
	cv::dilate(frame, frame_peak, kernel);
	cv::erode(frame, frame_flat, kernel);

	cv::Mat mask_peak, mask_flat, mask;
	cv::compare(frame, frame_peak, mask_peak, cv::CMP_GE);
	cv::compare(frame, frame_flat, mask_flat, cv::CMP_GT);
	cv::bitwise_and(mask_peak, mask_flat, mask);

	std::vector<cv::Point> points;
	cv::findNonZero(mask, points);
	const size_t num_points = points.size();
	keypoints.resize(num_points);
	for (size_t i = 0u; i < num_points; ++i)
	{
		const cv::Point& src = points[i];
		Feature& tgt = keypoints[i];
		tgt.x = static_cast<f32>(src.x);
		tgt.y = static_cast<f32>(src.y);
	}
#endif
	return true;
}

bool FeatureTracking::describeFeature_(
	const cv::Mat& frame,
	FeatureVec& features,
	cv::Mat& descriptors
)
{
	// Convert keypoints into OpenCV format	
	std::vector<cv::KeyPoint> keypoints_local;
	cv::KeyPoint::convert(features, keypoints_local);
	
	// Describe keypoints
#ifdef FEATURE_TRACKING_USE_ORB_FOR_FEATURE
	cv::Ptr<cv::ORB> describer = cv::ORB::create();
	describer->compute(frame, keypoints_local, descriptors);
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	cv::Ptr<cv::DescriptorExtractor> describer = cv::xfeatures2d::BriefDescriptorExtractor::create(16);		// 128[bit] (16[bytes]) is set here because of OpenCV implementation though 32[bit] in the original paper.
#else
	cv::Ptr<cv::DescriptorExtractor> describer = cv::BRISK::create(30, 0);
#endif
	describer->compute(frame, keypoints_local, descriptors);	// Keypoints close to borders will be modified; They will be rejected inside this function.
#endif

	// Convert keypoints into ours (Keypoints might be updated in the extractor)
	cv::KeyPoint::convert(keypoints_local, features);

	return true;
}

bool FeatureTracking::matchFeature_(
	const cv::Mat& descriptors_prev,
	const cv::Mat& descriptors_curr,
	DMatchVec& matches
)
{
	matches.clear();
	std::vector< DMatchVec > matches_best_two;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
	matcher->knnMatch(descriptors_prev, descriptors_curr, matches_best_two, 2);

	// Reject ambiguous matches
	const size_t num_match = matches_best_two.size();
	matches.reserve(num_match);
	for (size_t i = 0u; i < num_match; i++)
	{
		const DMatchVec& match_pair = matches_best_two[i];
		if (match_pair[0].distance < match_pair[1].distance * 0.5f)
		{
			matches.push_back(match_pair[0]);
		}
	}

#ifdef FEATURE_TRACKING_CONSOLE_LOG
	//std::cout << "  Matches: " << matches.size() << " (desc: " << descriptors_prev.rows << ", " << descriptors_curr.rows << ")" << std::endl;
#endif

	return true;
}

bool FeatureTracking::estimateDominantFlow_(
	const FeatureVec& features_prev,
	const FeatureVec& features_curr,
	const DMatchVec& matches,
	cv::Matx22f& A,
	Feature& b
)
{
	bool ret = false;

	// Extract corresponding points
	using FltT = f64;
	using Vec2 = cv::Matx<FltT, 2, 1>;
	const size_t num_in = matches.size();
	std::vector<Vec2> prev(num_in);
	std::vector<Vec2> curr(num_in);
	for (size_t i = 0u; i < num_in; ++i)
	{
		const cv::DMatch& match = matches[i];
		const i32& prev_idx = match.queryIdx;
		const i32& curr_idx = match.trainIdx;
		const Feature& kp_prev = features_prev[prev_idx];
		const Feature& kp_curr = features_curr[curr_idx];
		prev[i] << static_cast<FltT>(kp_prev.x), static_cast<FltT>(kp_prev.y);
		curr[i] << static_cast<FltT>(kp_curr.x), static_cast<FltT>(kp_curr.y);
	}

	// Estimate dominant flow
	const cv::Mat Ab = cv::estimateAffine2D(prev, curr, cv::noArray(), cv::RANSAC, 0.5, 2000u, 0.999, 10u);
	ret = !Ab.empty();
	if (ret)
	{
		A << static_cast<f32>(Ab.at<double>(0, 0)), static_cast<f32>(Ab.at<double>(0, 1)), static_cast<f32>(Ab.at<double>(1, 0)), static_cast<f32>(Ab.at<double>(1, 1));
		b.x = static_cast<f32>(Ab.at<double>(0, 2));
		b.y = static_cast<f32>(Ab.at<double>(1, 2));
	}
#ifdef FEATURE_TRACKING_CONSOLE_LOG
	std::cout << "A(cv):" << std::endl << A << std::endl;
	std::cout << "b(cv):" << std::endl << b << std::endl;
#endif
	return ret;
}

bool FeatureTracking::predictFlow_(
	const FeatureVec& features_prev,
	FeatureVec& features_curr_est,
	const cv::Matx22f& A,
	const Feature& b
)
{
	const size_t num_keypoints = features_prev.size();
	features_curr_est.resize(num_keypoints);
	for (size_t i = 0u; i < num_keypoints; ++i)
	{
		Feature& keypoint_curr_est = features_curr_est[i];
		const Feature& x = features_prev[i];
		keypoint_curr_est = A * x + b;
	}
	return true;
}

bool FeatureTracking::findLocalExtrema_(
	const FeatureVec& features_src,
	FeatureVec& features_dst,
	BoolVec& status,
	const cv::Mat& frame_curv,
	const f32& lambda,
	const f32& sigma,
	const i32& margin
)
{
	HillClimbing<f32> hc;
	hc.setLambda(lambda);
	hc.setSigma(sigma);
	hc.setMargin(margin);
	const bool ret = hc.setFrame(frame_curv);
	if (ret)
	{
		const size_t num_kp = features_src.size();
		features_dst.resize(num_kp);
		status.resize(num_kp);
		for (size_t i = 0u; i < num_kp; ++i)
		{
			const cv::Point kp_src = cv::Point(features_src[i]);
			cv::Point kp_dst;
			status[i] = hc.compute(kp_src, kp_dst);
			features_dst[i] = Feature(kp_dst);
		}
	}

	return ret;
}

bool FeatureTracking::refineFeature_(
	const cv::Mat& frame,
	FeatureVec& keypoints
)
{
	const cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
	cv::cornerSubPix(frame, keypoints, cv::Size(9, 9), cv::Size(-1, -1), criteria);
	return true;
}

bool FeatureTracking::extractFlow_(
	const FeatureVec& features_prev,
	const FeatureVec& features_curr,
	const BoolVec& status_curr,
	const FrameId prev_frame_id,
	const FrameId curr_frame_id,
	TrackIdVec& track_ids,
	KeypointVec& keypoints_total,
	TrackVec& tracks
)
{
	if ((features_prev.size() != features_curr.size())
		| (features_curr.size() != status_curr.size())
		| (features_curr.size() != track_ids.size()))
	{
		return false;
	}

	// Extract last keypoint ID and track ID
	KeypointId next_keypoint_id = (keypoints_total.empty() ? 0 : keypoints_total.back().keypoint_id);
	TrackId next_track_id = (tracks.empty() ? 0 : tracks.back().track_id + 1);

	// Extract successfull flow (tracked keypoints)
	auto ft_prev = features_prev.cbegin();
	auto ft_curr = features_curr.cbegin();
	auto stat_curr = status_curr.cbegin();
	auto track_id = track_ids.begin();
	for(size_t i = 0u; i < status_curr.size(); ++i)
	{
		if (*stat_curr)
		{
			// Merge track
			if (*track_id != INVALID_TRACK_ID)
			{
				// Add only current keypoint
				const Keypoint kp_curr(next_keypoint_id, curr_frame_id, *ft_curr);
				keypoints_total.push_back(kp_curr);

				// Add current keypoint ID into existing track
				auto track = std::find_if(tracks.begin(), tracks.end(),
					[&track_id](const Track& track) { return (track.track_id == *track_id); });
				if (track != tracks.end())
				{
					track->keypoint_ids.push_back(next_keypoint_id);
				}
			}
			else
			{
				// Add both previous and current keypoints
				const Keypoint kp_prev(next_keypoint_id, prev_frame_id, *ft_prev);
				const Keypoint kp_curr(next_keypoint_id + 1, curr_frame_id, *ft_curr);
				keypoints_total.push_back(kp_prev);
				keypoints_total.push_back(kp_curr);

				// Create new track
				Track track(next_track_id, kp_prev.keypoint_id, kp_curr.keypoint_id);
				tracks.push_back(track);
				*track_id = next_track_id;

				// Increment IDs
				++next_keypoint_id;		// Now same as current keypoint
				++next_track_id;
			}
		}
		else
		{
			// Discard track ID for next frame
			*track_id = INVALID_TRACK_ID;
		}

		// Increment ID and iterators
		++next_keypoint_id;
		++ft_prev;
		++ft_curr;
		++stat_curr;
		++track_id;
	}

	return true;
}

bool FeatureTracking::mergeFeature_(
	const FeatureVec& features_new,
	const FeatureVec& features_trk,
	const i32 width,
	const i32 height,
	const u8 nms_win_size,
	BoolVec& nms_table,
	TrackIdVec& track_ids,
	FeatureVec& features_merged
)
{
	if (features_trk.size() != track_ids.size())
	{
		return false;
	}

	// Initialize NMS table
	std::fill(nms_table.begin(), nms_table.end(), false);

	// Allocate outgoing buffer
	features_merged.clear();
	features_merged.reserve(features_trk.size() + features_new.size());
	TrackIdVec track_id_merged;
	track_id_merged.reserve(features_trk.size() + features_new.size());

#if 1
	// Register tracked feature points
	auto track_id = track_ids.cbegin();
	for (const auto& feature : features_trk)
	{
		if (*track_id != INVALID_TRACK_ID)
		{
			// Mark as "exist" in table
			updateNmsTable_(feature, width, height, nms_win_size, nms_table);

			// Output
			features_merged.push_back(feature);
			track_id_merged.push_back(*track_id);
		}
		++track_id;
	}
#endif

#if 1
	// Register new feature points
	for (const auto& feature : features_new)
	{
		// Check if inside image
		i32 idx = static_cast<i32>(std::round(feature.y)) * width + static_cast<i32>(std::round(feature.x));
		if (idx < 0 || static_cast<size_t>(idx) >= nms_table.size())
		{
			continue;
		}

		// Check if there are already feature points
		if (!nms_table[idx])
		{
			// Mark as "exist" in table
			updateNmsTable_(feature, width, height, nms_win_size, nms_table);

			// Output
			features_merged.push_back(feature);
			track_id_merged.push_back(INVALID_TRACK_ID);
		}
	}
#endif
	// Swap track IDs
	std::swap(track_id_merged, track_ids);

	return (features_merged.size() == track_ids.size());
}

void FeatureTracking::updateNmsTable_(
	const Feature& feature,
	const i32 width,
	const i32 height,
	const u8 nms_win_size,
	BoolVec& nms_table
)
{
	const u8 nms_win_size_half = nms_win_size >> 1;
	const i32 x_base = static_cast<i32>(std::round(feature.x)) - nms_win_size_half;
	i32 y = static_cast<i32>(std::round(feature.y)) - nms_win_size_half;
	i32 y_mul_width = y * width;
	for (u8 dy = 0u; dy < nms_win_size; ++dy)
	{
		if (y >= height)
		{
			break;
		}
		if (y >= 0)
		{
			i32 x = x_base;
			for (u8 dx = 0u; dx < nms_win_size; ++dx)
			{
				if (x >= width)
				{
					break;
				}
				if (x >= 0)
				{
					size_t idx = static_cast<size_t>(y_mul_width) + x;
					nms_table[idx] = true;
				}
				++x;
			}
		}
		++y;
		y_mul_width += width;
	}
}