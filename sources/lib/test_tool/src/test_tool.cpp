#include "test_tool.hpp"
#include <iostream>
#include <sstream>
#include <random>
#include <opencv2/imgproc.hpp>
#include "camera_mount.hpp"
#include "environment.hpp"
#include "environment_builder.hpp"
#include "mat_util.hpp"

PerspectiveCamera<f64> TestTool::genPerspectiveCamera()
{
	PerspectiveCameraParameter cam_param;
	cam_param.fx = 256.;
	cam_param.fy = 256.;
	cam_param.cx = 640.;
	cam_param.cy = 480.;
	cam_param.base.width = 1280;
	cam_param.base.height = 960;
	cam_param.base.setInitialized();
	cam_param.setInitialized();

	PerspectiveCamera<f64> cam;
	cam.setParameter(cam_param);
	static_cast<void>(cam.initialize());

	return cam;
}

bool TestTool::genTestDataset(const PerspectiveCamera<f64>& camera, const size_t num_cam, const size_t num_pt, TestDataset& test_dataset)
{
	KeypointVec& keypoints = test_dataset.keypoints;
	TrackVec& tracks = test_dataset.tracks;
	CameraPoseVec& camera_poses = test_dataset.camera_poses;
	PointVec& points = test_dataset.points;
	TargetIndices& indices = test_dataset.indices;

	// Define Camera mount
	CameraMount<f64> cam_mount(camera);
	cam_mount.setExtrinsics(cv::Point3d(0.0, 0.0, 0.5), cv::Vec3d(CV_PI / 2, 0., CV_PI / 2));

	// Define environment
	EnvironmentBuilder<f64> eb(cam_mount);
	using csPoint3 = CameraState<f64>::Point3;
	using csVec3 = CameraState<f64>::Vec3;
	eb.setNumOfWorldPoints(num_pt);
	eb.setROI(csPoint3(0.5, -20., -0.2), csPoint3(20., 20., 10.));
	eb.setNumOfCameras(num_cam);
	eb.setFps(10);
	eb.setInitCamPosition(csPoint3(0., 0., 0.));
	eb.setInitCamRotation(csPoint3(0., 0., 0.));
	eb.setCamPosVelocity(csVec3(0., 2.7778, 0.));
	eb.setCamPosAcceleration(csVec3(0., 0., 0.));
	eb.setCamRotVelocity(csVec3(0., 0., -0.31415));
	eb.setCamRotAcceleration(csVec3(0., 0., 0.));

	// Generate environment
	Environment<f64> env;
	bool ret = eb.build(env);

	if (ret)
	{
		// Copy keypoints and tracks
		{
			FrameId next_frame_id = 0;
			KeypointId next_keypoint_id = 0;
			for (const auto& cam_stat : env.camera_states)
			{
				const std::vector<ImagePointState<f64> >& img_points = cam_stat.img_points;
				keypoints.reserve(keypoints.size() + img_points.size());
				for (const auto& img_pt : img_points)
				{
					// Keypoints
					const ImagePointState<f64>::Point2& coord = img_pt.coord;
					cv::Point2f coord_cast(static_cast<f32>(coord.x), static_cast<f32>(coord.y));
					Keypoint keypoint(next_keypoint_id, next_frame_id, coord_cast);
					keypoints.push_back(keypoint);

					// Tracks
					auto track_it = std::find_if(tracks.begin(), tracks.end(),
						[&img_pt](const Track& trk) { return (trk.track_id == static_cast<TrackId>(img_pt.index_world)); });
					if (track_it == tracks.end())
					{
						// New
						Track track;
						track.track_id = static_cast<TrackId>(img_pt.index_world);
						track.keypoint_ids.push_back(next_keypoint_id);
						tracks.push_back(track);
					}
					else
					{
						track_it->keypoint_ids.push_back(next_keypoint_id);
					}

					++next_keypoint_id;
				}
				++next_frame_id;
			}
		}

		// Copy camera poses
		{
			FrameId next_frame_id = 0;
			camera_poses.resize(env.camera_states.size());
			for (size_t i = 0u; i < camera_poses.size(); ++i)
			{
				const CameraState<f64>& cam_state = env.camera_states[i];
				CameraPose& cam_pose = camera_poses[i];
				cam_pose.translation = cam_state.position;
				cam_pose.rotation = cam_state.rotation;
				cam_pose.frame_id = next_frame_id;
				++next_frame_id;
			}
		}

		// Copy points
		{
			points.resize(env.world_points.size());
			for (size_t i = 0u; i < points.size(); ++i)
			{
				const WorldPointState<f64>& wp = env.world_points[i];
				Point& pt = points[i];
				pt.coord(0) = wp.coord.x;
				pt.coord(1) = wp.coord.y;
				pt.coord(2) = wp.coord.z;
				pt.track_id = static_cast<TrackId>(wp.index);
				pt.status = Point::Status::OK;
			}
		}

		// Generate indices
		{
			indices.target_frame_ids.reserve(camera_poses.size());
			for (auto cam_pose = camera_poses.crbegin(); cam_pose != camera_poses.crend(); ++cam_pose)
			{
				indices.target_frame_ids.push_back(cam_pose->frame_id);
			}

			indices.target_track_ids.reserve(points.size());
			for (const auto& point : points)
			{
				indices.target_track_ids.push_back(point.track_id);
			}
		}
	}

	return ret;
}

bool TestTool::genTestFrames(const FeatureBuffer& feature, const CameraParameter& cam_param, const f64 fps, std::vector<Frame>& frames)
{
	// Check if given fps is zero or negative
	if (fps < std::numeric_limits<f64>::epsilon())
	{
		return false;
	}

	// Generate frames
	for (const auto& track : feature.tracks)
	{
		for (const auto& keypoint_id : track.keypoint_ids)
		{
			// Extract keypoint ID
			auto keypoint = std::find_if(feature.keypoints.cbegin(), feature.keypoints.cend(),
				[&keypoint_id](const Keypoint& kp) { return (kp.keypoint_id == keypoint_id); });
			if ((keypoint == feature.keypoints.cend())
				|| (keypoint->frame_id < 0))
			{
				continue;
			}

			// Check if corresponding frame already exists or not
			Image img;
			const FrameId frame_id = keypoint->frame_id;
			auto frame = std::find_if(frames.begin(), frames.end(),
				[&frame_id](const Frame& f) { return (f.frame_id == frame_id); });
			if (frame == frames.end())
			{
				// Create a new frame
				Frame new_frame;
				new_frame.img = Image(cv::Size(cam_param.width, cam_param.height), CV_8UC3, cv::Scalar::all(128.0));
				new_frame.frame_id = frame_id;
				new_frame.timestamp = frame_id / fps;
				frames.push_back(new_frame);
				img = frames.back().img;
			}
			else
			{
				// Get the reference to image
				img = frame->img;
			}

			// Draw
			cv::Scalar color(
				85.0 * (track.track_id % 4),
				85.0 * (track.track_id / 4 % 4),
				85.0 * (track.track_id / 16 % 4)
			);
			cv::circle(img, keypoint->coord, 4, color, cv::FILLED);
		}
	}

	return true;
}

void TestTool::addNoiseToFeatureBuffer(TestDataset& test_dataset)
{
	static_cast<void>(test_dataset);
	// TODO: IMPLEMENT ME
}

void TestTool::addNoiseToWorldBuffer(TestDataset& test_dataset, const bool add_to_camera_poses, const bool add_to_points)
{
	// Prepare random engine
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());
	std::normal_distribution<f64> dist_rot_rad(0.0, 0.01);
	std::normal_distribution<f64> dist_trans_m(0.0, 0.1);
	std::normal_distribution<f64> dist_pt_m(0.0, 0.5);

	// Add noise to camera poses
	if (add_to_camera_poses)
	{
		for (size_t i = 0u; i < test_dataset.camera_poses.size(); ++i)
		{
			CameraPose& cam_pose = test_dataset.camera_poses[i];
			const f64 noises[6] = {
				// Rotation [rad]
				dist_rot_rad(engine),
				dist_rot_rad(engine),
				dist_rot_rad(engine),

				// Translation [m]
				dist_trans_m(engine),
				dist_trans_m(engine),
				dist_trans_m(engine)
			};
			cam_pose.rotation(0) += noises[0];
			cam_pose.rotation(1) += noises[1];
			cam_pose.rotation(2) += noises[2];
			cam_pose.translation(0) += noises[3];
			cam_pose.translation(1) += noises[4];
			cam_pose.translation(2) += noises[5];
			std::cout << "Noise: (" << noises[0] << ", " << noises[1] << ", " << noises[2] << ")[rad], (" << noises[3] << ", " << noises[4] << ", " << noises[5] << ")[m]" << std::endl;
		}
	}

	// Add noise to 3D points
	if (add_to_points)
	{
		for (size_t i = 0u; i < test_dataset.points.size(); ++i)
		{
			Point& point = test_dataset.points[i];
			const f64 noises[3] = {
				dist_pt_m(engine),
				dist_pt_m(engine),
				dist_pt_m(engine)
			};
			point.coord(0) += noises[0];
			point.coord(1) += noises[1];
			point.coord(2) += noises[2];
			std::cout << "Noise: (" << noises[0] << ", " << noises[1] << ", " << noises[2] << ")" << std::endl;
		}
	}
}

std::string TestTool::compareTestDatasets(const TestDataset& data_a, const TestDataset& data_b, const bool explain_camera_poses, const bool explain_points)
{
	std::stringstream ss;
	if (data_a.camera_poses.size() != data_b.camera_poses.size()
		|| data_a.points.size() != data_b.points.size())
	{
		std::cerr << "ERROR: Number of elements are different." << std::endl;
		return ss.str();
	}

	ss << "Compared \"" << data_a.name << "\" with \"" << data_b.name <<  "\"";

	// Explain error of camera poses
	if (explain_camera_poses)
	{
		f64 total_diff_trans = 0.0;
		f64 total_diff_rad = 0.0;
		for (size_t i = 0u; i < data_a.camera_poses.size(); ++i)
		{
			const CameraPose& cam_pose_a = data_a.camera_poses[i];
			const CameraPose& cam_pose_b = data_b.camera_poses[i];
			const Vec3 diff_trans(cam_pose_a.translation - cam_pose_b.translation);
			Mat33 rot_a, rot_b;
			MatUtil<f64>::genRotationMatrix(cam_pose_a.rotation, rot_a);
			MatUtil<f64>::genRotationMatrix(cam_pose_b.rotation, rot_b);
			const Vec3 diff_rot(cam_pose_a.rotation - cam_pose_b.rotation);
			const Vec3 unit_vec(1.f, 0.f, 0.f);
			const f64 diff_trans_norm = cv::norm(diff_trans);
			const f64 diff_rad = std::acos((rot_a * unit_vec).dot(rot_b * unit_vec));
			ss << std::endl << "  Cam-" << i << "> Translation [m]: " << diff_trans.t() << " (" << diff_trans_norm << " [m])"
				<< std::endl << "          Rotation [rad]: " << diff_rot.t() << " (" << diff_rad << " [rad])";
			total_diff_trans += diff_trans_norm;
			total_diff_rad += diff_rad;
		}
		ss << std::endl << "  * Total: " << total_diff_trans << "[m], " << total_diff_rad << "[rad]";
	}

	// Explain error of points
	if (explain_points)
	{
		f64 total_diff = 0.0;
		for (size_t i = 0u; i < data_a.points.size(); ++i)
		{
			const Vec3& point3d_a = data_a.points[i].coord;
			const Vec3& point3d_b = data_b.points[i].coord;
			const Vec3 diff(point3d_a - point3d_b);
			const f64 diff_norm = cv::norm(diff);
			ss << std::endl << "  Point3d-" << i << "> " << diff.t() << " (" << diff_norm << " [m])";
			total_diff += diff_norm;
		}
		ss << std::endl << "  * Total: " << total_diff << "[m]";
	}

	return ss.str();
}