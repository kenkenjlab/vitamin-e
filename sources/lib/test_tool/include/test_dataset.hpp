#ifndef TEST_DATASETE_HPP
#define TEST_DATASETE_HPP

#include "feature_buffer.hpp"
#include "world_buffer.hpp"
#include "target_indices.hpp"

struct TestDataset
{
	std::string name;
	KeypointVec keypoints;
	TrackVec tracks;
	CameraPoseVec& camera_poses;
	PointVec& points;
	TargetIndices indices;

	explicit TestDataset(const std::string& _name = "undefined")
		: name(_name)
		, camera_poses(world_buff_.camera_poses)
		, points(world_buff_.points)
		, world_buff_(CameraPoseVec(), PointVec())
	{}
	
	explicit TestDataset(const TestDataset& dataset, const std::string& _name)
		: name(_name)
		, keypoints(dataset.keypoints)
		, tracks(dataset.tracks)
		, camera_poses(world_buff_.camera_poses)
		, points(world_buff_.points)
		, indices(dataset.indices)
		, world_buff_(dataset.camera_poses, dataset.points)
	{}

	inline FeatureBuffer getFeatureBuffer() const { return FeatureBuffer(keypoints, tracks); }

	inline WorldBuffer& getWorldBuffer() { return world_buff_; }
	
	inline const WorldBuffer& getWorldBuffer() const { return world_buff_; }
	
	std::string str(const i8 verbose_keypoints = 0, const i8 verbose_tracks = 0, const bool verbose_cameras = false, const bool verbose_points = false, const std::string& prefix = "") const;

private:
	WorldBuffer world_buff_;
};

#endif