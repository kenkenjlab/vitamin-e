#include <iostream>
#include "test_tool.hpp"
#include "point_subspace_gauss_newton.hpp"
#include "camera_pose_subspace_gauss_newton.hpp"
#include "simultaneous_subspace_gauss_newton.hpp"
#include "world_visualizer.hpp"

int testOnlyPointOptimization(const TestDataset& gt, const PerspectiveCamera<f64>& cam);
int testOnlyCameraPoseOptimization(const TestDataset& gt, const PerspectiveCamera<f64>& cam);
int testBothOptimizations(const TestDataset& gt, const PerspectiveCamera<f64>& cam);

int main(int argc, char** argv)
{
	// Parse arguments
	i32 mode = 0;
	if (argc > 1)
	{
		mode = std::atoi(argv[1]);
	}

	// Prepare camera
	const PerspectiveCamera<f64> cam = TestTool::genPerspectiveCamera();
	if (!cam.isInitialized())
	{
		std::cerr << "ERROR: Failed to initialize camera" << std::endl;
		return -1;
	}

	// Generate ground truth
	TestDataset gt("Ground Truth");
	if (!TestTool::genTestDataset(cam, OPTIMIZER_MAX_NUM_CAM, OPTIMIZER_MAX_NUM_PT, gt))
	{
		std::cerr << "ERROR: Failed to generate test dataset" << std::endl;
		return -2;
	}

	// Run test
	int ret = 0;
	switch (mode)
	{
	default:
	case 0:
		ret = testOnlyPointOptimization(gt, cam);
		break;
	case 1:
		ret = testOnlyCameraPoseOptimization(gt, cam);
		break;
	case 2:
		ret = testBothOptimizations(gt, cam);
		break;
	}

	return ret;
}

int testOnlyPointOptimization(const TestDataset& gt, const PerspectiveCamera<f64>& cam)
{
	// Generate initial guess
	TestDataset init_guess(gt, "Good camera poses, Bad points");
#if 0
	TestTool::addNoiseToWorldBuffer(init_guess, false, true);
#elif 0
	const CameraPose& latest_cam_pose = init_guess.camera_poses.back();
	for (Point& pt : init_guess.points)
	{
		pt.coord = latest_cam_pose.getRotationMatrix() * Vec3(0.0, 0.0, 1.0) + latest_cam_pose.translation;
	}
#else
	// TEMP: Reconstruct by DLT
	{
		for (const Track& track : init_guess.tracks)
		{
			// Check number of keypoints
			if (track.keypoint_ids.size() < 2u)
			{
				continue;
			}

			// Extract corresponding point
			auto point = std::find_if(init_guess.points.begin(), init_guess.points.end(),
				[&track](const Point& pt) { return (pt.track_id == track.track_id); });
			if (point == init_guess.points.end())
			{
				continue;
			}

			// Extract corresponding keypoints
			auto kp1 = std::find_if(init_guess.keypoints.cbegin(), init_guess.keypoints.cend(),
				[&track](const Keypoint& kp) { return (kp.keypoint_id == track.keypoint_ids.front()); });
			auto kp2 = std::find_if(init_guess.keypoints.cbegin(), init_guess.keypoints.cend(),
				[&track](const Keypoint& kp) { return (kp.keypoint_id == track.keypoint_ids.back()); });
			if (kp1 == init_guess.keypoints.cend())
			{
				continue;
			}
			if (kp2 == init_guess.keypoints.cend())
			{
				continue;
			}

			// Generate projection matrices
			auto cam_pose1 = std::find_if(init_guess.camera_poses.cbegin(), init_guess.camera_poses.cend(),
				[&kp1](const CameraPose& cam_pose) { return (cam_pose.frame_id == kp1->frame_id); });
			auto cam_pose2 = std::find_if(init_guess.camera_poses.cbegin(), init_guess.camera_poses.cend(),
				[&kp2](const CameraPose& cam_pose) { return (cam_pose.frame_id == kp2->frame_id); });
			if (cam_pose1 == init_guess.camera_poses.cend())
			{
				continue;
			}
			if (cam_pose2 == init_guess.camera_poses.cend())
			{
				continue;
			}
			const Mat33 R1 = cam_pose1->getRotationMatrix();
			const Mat33 R2 = cam_pose2->getRotationMatrix();
			Mat33 K;
			Mat34 Rt1, Rt2;
			Mat34 P1, P2;
			const PerspectiveCameraParameter& pers_cam_param = cam.getParameter();
			K << pers_cam_param.fx, 0.0, pers_cam_param.cx,
				0.0, pers_cam_param.fy, pers_cam_param.cy,
				0.0, 0.0, 1.0;
			Rt1 << R1(0, 0), R1(0, 1), R1(0, 2), cam_pose1->translation(0),
				R1(1, 0), R1(1, 1), R1(1, 2), cam_pose1->translation(1),
				R1(2, 0), R1(2, 1), R1(2, 2), cam_pose1->translation(2);
			Rt2 << R2(0, 0), R2(0, 1), R2(0, 2), cam_pose2->translation(0),
				R2(1, 0), R2(1, 1), R2(1, 2), cam_pose2->translation(1),
				R2(2, 0), R2(2, 1), R2(2, 2), cam_pose2->translation(2);
			P1 = K * Rt1;
			P2 = K * Rt2;
#if 1
			// http://daily-tech.hatenablog.com/entry/2019/07/15/183302
			Mat43 A;
			A << (kp1->coord.x * P1(2, 0) - P1(0, 0)), (kp1->coord.x * P1(2, 1) - P1(0, 1)), (kp1->coord.x * P1(2, 2) - P1(0, 2)),
				(kp1->coord.y * P1(2, 0) - P1(1, 0)), (kp1->coord.y * P1(2, 1) - P1(1, 1)), (kp1->coord.y * P1(2, 2) - P1(1, 2)),
				(kp2->coord.x * P2(2, 0) - P2(0, 0)), (kp2->coord.x * P2(2, 1) - P2(0, 1)), (kp2->coord.x * P2(2, 2) - P2(0, 2)),
				(kp2->coord.y * P2(2, 0) - P2(1, 0)), (kp2->coord.y * P2(2, 1) - P2(1, 1)), (kp2->coord.y * P2(2, 2) - P2(1, 2));
			Vec4 b;
			b << -(kp1->coord.x * P1(2, 3) - P1(0, 3)),
				-(kp1->coord.y * P1(2, 3) - P1(1, 3)),
				-(kp2->coord.x * P2(2, 3) - P2(0, 3)),
				-(kp2->coord.y * P2(2, 3) - P2(1, 3));
			Vec3 X;
#else
			// http://bardsley.org.uk/wp-content/uploads/2007/02/3d-reconstruction-using-the-direct-linear-transform.pdf
			// https://www.uio.no/studier/emner/matnat/its/nedlagte-emner/UNIK4690/v16/forelesninger/lecture_7_2-triangulation.pdf
			Mat44 A;
			A << (kp1->coord.x * P1(2, 0) - P1(0, 0)), (kp1->coord.x * P1(2, 1) - P1(0, 1)), (kp1->coord.x * P1(2, 2) - P1(0, 2)), (kp1->coord.x * P1(2, 3) - P1(0, 3)),
				(kp1->coord.y * P1(2, 0) - P1(1, 0)), (kp1->coord.y * P1(2, 1) - P1(1, 1)), (kp1->coord.y * P1(2, 2) - P1(1, 2)), (kp1->coord.y * P1(2, 3) - P1(1, 3)),
				(kp2->coord.x * P2(2, 0) - P2(0, 0)), (kp2->coord.x * P2(2, 1) - P2(0, 1)), (kp2->coord.x * P2(2, 2) - P2(0, 2)), (kp2->coord.x * P2(2, 3) - P2(0, 3)),
				(kp2->coord.y * P2(2, 0) - P2(1, 0)), (kp2->coord.y * P2(2, 1) - P2(1, 1)), (kp2->coord.y * P2(2, 2) - P2(1, 2)), (kp2->coord.y * P2(2, 3) - P2(1, 3));
			Vec4 b(Vec4::zeros());
			Vec4 X;
#endif
			// Reconstruct
			bool ret = cv::solve(A, b, X, cv::DECOMP_SVD);
			if (ret)
			{
				point->coord(0) = X(0);
				point->coord(1) = X(1);
				point->coord(2) = X(2);
			}
		}
	}

#endif
	TestDataset final_guess(init_guess, "Optimized points");
	FeatureBuffer feat_buff = final_guess.getFeatureBuffer();
	WorldBuffer& world_buff = final_guess.getWorldBuffer();

	// Optimize
	PointSubspaceGaussNewton<OptFltT> solver;
	solver.setCamera(cam);
	bool ret = solver.compute(feat_buff, final_guess.indices, world_buff);
	if (!ret)
	{
		std::cerr << "Failed to run" << std::endl;
		return 1;
	}

	// Evaluate
	std::cout << "========" << std::endl;
	std::cout << gt.str(1, 0, true, true) << std::endl;
	std::cout << "========" << std::endl;
	std::cout << init_guess.str(0, 0, false, true) << std::endl;
	std::cout << "----" << std::endl;
	std::cout << TestTool::compareTestDatasets(init_guess, gt, true, true) << std::endl;
	std::cout << "========" << std::endl;
	std::cout << final_guess.str(0, 0, false, true) << std::endl;
	std::cout << "----" << std::endl;
	std::cout << TestTool::compareTestDatasets(final_guess, gt, true, true) << std::endl;
	std::cout << "========" << std::endl;

	// Visualize
	std::vector<Frame> frames;
	const PerspectiveCameraParameter cam_param = cam.getParameter();
	static_cast<void>(TestTool::genTestFrames(gt.getFeatureBuffer(), cam_param.base, 30.0, frames));
	WorldVisualizer viewer("Point optimization");
	viewer.addAxis(Mat44::eye(), "origin", 1.0);
	viewer.addCameras(gt.camera_poses, frames, cam_param, "camera_gt", 0.2, cv::viz::Color::blue());
	viewer.addPoints(gt.points, "points_gt", cv::viz::Color::green());
	viewer.addPoints(init_guess.points, "points_init", cv::viz::Color::orange());
	viewer.addPoints(final_guess.points, "points_final", cv::viz::Color::red());
	viewer.addPointCorrespondences(init_guess.points, final_guess.points, "optimized_distance", cv::viz::Color::gray());
	viewer.addPointCorrespondences(gt.points, final_guess.points, "error_distance", cv::viz::Color::white());
	viewer.spin();

	return 0;
}

int testOnlyCameraPoseOptimization(const TestDataset& gt, const PerspectiveCamera<f64>& cam)
{
	// Generate initial guess
	TestDataset init_guess(gt, "Good points, Bad camera poses");
#if 0
	TestTool::addNoiseToWorldBuffer(init_guess, true, false);
#else
	const CameraPose& first_cam_pose = init_guess.camera_poses.front();
	for (CameraPose& cam_pose : init_guess.camera_poses)
	{
		cam_pose.rotation = first_cam_pose.rotation;
		cam_pose.translation = first_cam_pose.translation;
	}
#endif
	TestDataset final_guess(init_guess, "Optimized camera poses");
	FeatureBuffer feat_buff = final_guess.getFeatureBuffer();
	WorldBuffer& world_buff = final_guess.getWorldBuffer();

	// Optimize
	CameraPoseSubspaceGaussNewton<OptFltT> solver;
	solver.setCamera(cam);
	bool ret = solver.compute(feat_buff, final_guess.indices, world_buff);
	if (!ret)
	{
		std::cerr << "Failed to run" << std::endl;
		return 1;
	}

	// Evaluate
	std::cout << "========" << std::endl;
	std::cout << gt.str(1, 0, true, true) << std::endl;
	std::cout << "========" << std::endl;
	std::cout << init_guess.str(0, 0, true, false) << std::endl;
	std::cout << "----" << std::endl;
	std::cout << TestTool::compareTestDatasets(init_guess, gt, true, true) << std::endl;
	std::cout << "========" << std::endl;
	std::cout << final_guess.str(0, 0, true, false) << std::endl;
	std::cout << "----" << std::endl;
	std::cout << TestTool::compareTestDatasets(final_guess, gt, true, true) << std::endl;
	std::cout << "========" << std::endl;

	// Visualize
	std::vector<Frame> frames;
	const PerspectiveCameraParameter cam_param = cam.getParameter();
	static_cast<void>(TestTool::genTestFrames(gt.getFeatureBuffer(), cam_param.base, 30.0, frames));
	WorldVisualizer viewer("Camera pose optimization");
	viewer.addAxis(Mat44::eye(), "origin", 1.0);
	const f64 cam_scale = 0.2;
	viewer.addCameras(gt.camera_poses, frames, cam_param, "camera_gt", cam_scale, cv::viz::Color::blue());
	viewer.addCameras(init_guess.camera_poses, frames, cam_param, "camposes_init", cam_scale, cv::viz::Color::orange());
	viewer.addCameras(final_guess.camera_poses, frames, cam_param, "camposes_final", cam_scale, cv::viz::Color::red());
	viewer.addPoints(gt.points, "points_gt", cv::viz::Color::green());
	viewer.addCameraPoseCorrespondences(init_guess.camera_poses, final_guess.camera_poses, "optimized_distance", cv::viz::Color::gray());
	viewer.addCameraPoseCorrespondences(gt.camera_poses, final_guess.camera_poses, "error_distance", cv::viz::Color::white());
	viewer.spin();

	return 0;
}

int testBothOptimizations(const TestDataset& gt, const PerspectiveCamera<f64>& cam)
{
	// Generate initial guess
	TestDataset init_guess(gt, "Bad points, Bad camera poses");
	TestTool::addNoiseToWorldBuffer(init_guess, true, false);
	TestDataset final_guess(init_guess, "Optimized camera poses");
	FeatureBuffer feat_buff = final_guess.getFeatureBuffer();
	WorldBuffer& world_buff = final_guess.getWorldBuffer();

	// Optimize camera poses and points
	SimultaneousSubspaceGaussNewton<OptFltT> solver;
	solver.setCamera(cam);
	bool ret = solver.compute(feat_buff, final_guess.indices, world_buff);
	if (!ret)
	{
		std::cerr << "Failed to run" << std::endl;
		return 1;
	}

	// Evaluate
	std::cout << "========" << std::endl;
	std::cout << gt.str(1, 0, true, true) << std::endl;
	std::cout << "========" << std::endl;
	std::cout << init_guess.str(0, 0, true, true) << std::endl;
	std::cout << "----" << std::endl;
	std::cout << TestTool::compareTestDatasets(init_guess, gt, true, true) << std::endl;
	std::cout << "========" << std::endl;
	std::cout << final_guess.str(0, 0, true, true) << std::endl;
	std::cout << "----" << std::endl;
	std::cout << TestTool::compareTestDatasets(final_guess, gt, true, true) << std::endl;
	std::cout << "========" << std::endl;

	// Visualize
	std::vector<Frame> frames;
	const PerspectiveCameraParameter cam_param = cam.getParameter();
	static_cast<void>(TestTool::genTestFrames(gt.getFeatureBuffer(), cam_param.base, 30.0, frames));
	WorldVisualizer viewer("Camera pose optimization");
	viewer.addAxis(Mat44::eye(), "origin", 1.0);
	const f64 cam_scale = 0.2;
	viewer.addCameras(gt.camera_poses, frames, cam_param, "camera_gt", cam_scale, cv::viz::Color::blue());
	viewer.addCameras(init_guess.camera_poses, frames, cam_param, "camposes_init", cam_scale, cv::viz::Color::orange());
	viewer.addCameras(final_guess.camera_poses, frames, cam_param, "camposes_final", cam_scale, cv::viz::Color::red());
	viewer.addPoints(gt.points, "points_gt", cv::viz::Color::green());
	viewer.addPoints(init_guess.points, "points_init", cv::viz::Color::orange());
	viewer.addPoints(final_guess.points, "points_final", cv::viz::Color::red());
	viewer.addCameraPoseCorrespondences(init_guess.camera_poses, final_guess.camera_poses, "cam_optimized_distance", cv::viz::Color::gray());
	viewer.addCameraPoseCorrespondences(gt.camera_poses, final_guess.camera_poses, "cam_error_distance", cv::viz::Color::white());
	viewer.addPointCorrespondences(init_guess.points, final_guess.points, "pt_optimized_distance", cv::viz::Color::gray());
	viewer.addPointCorrespondences(gt.points, final_guess.points, "pt_error_distance", cv::viz::Color::white());
	viewer.spin();

	return 0;
}