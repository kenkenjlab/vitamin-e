#include <iostream>
#include "test_tool.hpp"
#include "camera_pose_initial_estimation.hpp"
#include "world_visualizer.hpp"

int testCameraPoseInitialEstimation(const TestDataset& gt, const PerspectiveCamera<f64>& cam);

int main(void)
{
	// Prepare camera
	const PerspectiveCamera<f64> cam = TestTool::genPerspectiveCamera();
	if (!cam.isInitialized())
	{
		std::cerr << "ERROR: Failed to initialize camera" << std::endl;
		return -1;
	}

	// Generate ground truth
	TestDataset gt("Ground Truth");
	if (!TestTool::genTestDataset(cam, 2u, 1000u, gt))
	{
		std::cerr << "ERROR: Failed to generate test dataset" << std::endl;
		return -2;
	}
	
	// Normalize scale
	const Vec3& first_cam_pos = gt.camera_poses[1].translation;
	const Vec3& second_cam_pos = gt.camera_poses[0].translation;
	const CamPoseFltT distance = cv::norm(second_cam_pos - first_cam_pos);
	const CamPoseFltT distance_inv = 1.0 / distance;
	for (auto& pt : gt.points)
	{
		pt.coord *= distance_inv;
	}
	for (auto& cam_pose : gt.camera_poses)
	{
		cam_pose.translation *= distance_inv;
	}

	// Run test
	int ret = testCameraPoseInitialEstimation(gt, cam);

	return ret;
}

int testCameraPoseInitialEstimation(const TestDataset& gt, const PerspectiveCamera<f64>& cam)
{
	// Generate final guess except camera poses
	TestDataset final_guess(gt, "Optimized camera poses");
	final_guess.camera_poses.clear();
	FeatureBuffer feat_buff = final_guess.getFeatureBuffer();

	// Estimate in first camera coordinate
	CameraPoseInitialEstimation estimator;
	CameraPoseInitialEstimationParameter param;
	estimator.setParameter(param, cam.getParameter());
	CameraPoseVec cam_poses;
	bool ret = estimator.compute(feat_buff.keypoints, feat_buff.tracks, 0, 1, cam_poses);
	if (!ret)
	{
		std::cerr << "Failed to run" << std::endl;
		return 1;
	}

	// Transform from first camera coordinate to world coordinate
	if (!gt.camera_poses.empty())
	{
		const Mat33 R_cam2mnt = gt.camera_poses.front().getRotationMatrix();
		const Vec3 t_cam2mnt = gt.camera_poses.front().translation;
		for (auto& cam_pose : cam_poses)
		{

			const Mat33 R = cam_pose.getRotationMatrix();
			const Vec3 t = -cam_pose.translation;
			cam_pose.setRotationMatrix(R_cam2mnt * R);
			cam_pose.translation = R_cam2mnt * t + t_cam2mnt;
		}
	}
	final_guess.camera_poses = cam_poses;

	// Evaluate
	std::cout << "========" << std::endl;
	std::cout << gt.str(0, 0, true, false) << std::endl;
	std::cout << "========" << std::endl;
	std::cout << final_guess.str(0, 0, true, false) << std::endl;
	std::cout << "----" << std::endl;
	std::cout << TestTool::compareTestDatasets(final_guess, gt, true, false) << std::endl;
	std::cout << "========" << std::endl;

	// Visualize
	std::vector<Frame> frames;
	const PerspectiveCameraParameter cam_param = cam.getParameter();
	static_cast<void>(TestTool::genTestFrames(gt.getFeatureBuffer(), cam_param.base, 30.0, frames));
	WorldVisualizer viewer("Camera pose optimization");
	viewer.addAxis(Mat44::eye(), "origin", 1.0);
	const f64 cam_scale = 0.2;
	viewer.addCameras(gt.camera_poses, frames, cam_param, "camera", cam_scale, cv::viz::Color::blue());
	viewer.addCameras(final_guess.camera_poses, frames, cam_param, "camposes_final", cam_scale, cv::viz::Color::red());
	viewer.addPoints(gt.points, "points_gt", cv::viz::Color::green());
	viewer.addCameraPoseCorrespondences(gt.camera_poses, final_guess.camera_poses, "error_distance", cv::viz::Color::white());
	viewer.spin();

	return 0;
}
