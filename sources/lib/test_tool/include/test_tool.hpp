#ifndef TEST_TOOl_HPP
#define TEST_TOOl_HPP

#include "definition.hpp"
#include "perspective_camera.hpp"
#include "perspective_camera_parameter.hpp"
#include "test_dataset.hpp"
#include "frame.hpp"

class TestTool
{
public:
	static PerspectiveCamera<f64> genPerspectiveCamera();
	static bool genTestDataset(const PerspectiveCamera<f64>& camera, const size_t num_cam, const size_t num_pt, TestDataset& test_dataset);
	static bool genTestFrames(const FeatureBuffer& feature, const CameraParameter& cam_param, const f64 fps, std::vector<Frame>& frames);
	static void addNoiseToFeatureBuffer(TestDataset& test_dataset);
	static void addNoiseToWorldBuffer(TestDataset& test_dataset, const bool add_to_camera_poses, const bool add_to_points);
	static std::string compareTestDatasets(const TestDataset& data_a, const TestDataset& data_b, const bool explain_camera_poses, const bool explain_points);
};

#endif