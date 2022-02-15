#pragma once

#include <vector>
#include <cmath>
#include <boost/thread.hpp>
#include "callbacks.h"
#include "obstructable_grid.h"
#include "slam_message.h"

using NTNU::graph::grid::obstructable_grid;
using NTNU::graph::grid::vertex_set;

namespace NTNU::application::SLAM
{

struct cell_t {
	int16_t row;
	int16_t col;

	inline bool operator == (const cell_t a) {
		return (row == a.row && col == a.col);
	}
	inline cell_t operator + (const cell_t a) {
		return cell_t({ row + a.row, col + a.col});
	}
};

struct pose_t {
	double x;
	double y;
	double theta;

	inline bool operator != (const pose_t a) {
		return !(x == a.x && y == a.y && theta == a.theta);
	}

	inline bool operator == (const pose_t a) {
		return (x == a.x && y == a.y && theta == a.theta);
	}

	inline pose_t operator + (const pose_t a) {
		return pose_t({ x + a.x, y + a.y, theta + a.theta });
	}

	inline pose_t operator - (const pose_t a) {
		return pose_t({ x - a.x, y - a.y, theta - a.theta });
	}

	inline pose_t operator * (const double a) {
		return pose_t({ x * a, y * a, theta * a });
	}

	inline double operator * (const pose_t a) {
		return x * a.x + y * a.y + theta * a.theta;
	}

	inline pose_t operator / (const double a) {
		return pose_t({ x / a, y / a, theta / a });
	}

	inline bool operator >= (const pose_t a) {
		return (abs(x) >= abs(a.x) || abs(y) >= abs(a.y));
	}

};

struct Particle {
	int16_t id;
	pose_t pose;
	double weight;
	obstructable_grid map = obstructable_grid(500, 500); //rows, cols
	std::vector<std::pair<cell_t,int>> raster_array;

	Particle(int16_t id_p, pose_t pose_p, double weight_p, obstructable_grid map_p);
	Particle(int16_t id_p, pose_t pose_p, double weight_p);
};

class particleFilter {
public:
	particleFilter(std::string robot, pose_t init_pose, std::array<double,4> a, std::array<int16_t,4> settings);

	virtual ~particleFilter() {};

	void estimate_pose(Particle& p, pose_t& odom);
	void update_particle(pose_t odom, std::vector<message::position> obstacles, std::vector<bool> is_object);
	void resample_particles();
	pose_t getPose();
	obstructable_grid getBestMap();
	double get_random(double mean, double std);
	void cleanRaster();
	//void search_for_transform(const int &i);// Particle* p, std::vector<message::position> points, pose_t pose_guess, std::vector<bool> is_object, std::pair<pose_t, int>& best_trans_weight, std::vector<std::pair<cell_t, int>>& best_raster_array);// boost::shared_ptr<std::pair<pose_t, int>>& best_trans_weight, boost::shared_ptr<std::vector<std::pair<cell_t, int>>>& best_raster_array, int i);

private:
	void search_for_transform(Particle& p, std::vector<message::position>& points, pose_t& pose_guess, std::vector<bool>& is_object, std::array<int16_t, 2>& size, int16_t& separation, std::vector<std::pair<cell_t, int>>& local_raster_array, std::vector<std::pair<pose_t, int>>& best_trans);
	double correlation_model(Particle& p, std::vector<message::position>& points, std::vector<bool>& is_object);
	double correlation_model(Particle& p, pose_t& pose, std::vector<message::position>& points, std::vector<bool>& is_object);
	std::vector<int16_t> systematic_resample(std::vector<double>& weights);
	void integrate_scan(Particle& p, std::vector<message::position>& points, std::vector<bool>& is_object);
	//std::optional<std::pair<pose_t, int>> scan_match(Particle& p, std::vector<message::position>& points, pose_t& pose_guess, std::vector<bool>& is_object, std::vector<std::pair<cell_t, int>>& local_raster_array);
	std::optional<std::vector<pose_t>> scan_match(Particle& p, std::vector<message::position>& points, pose_t& pose_guess, std::vector<bool>& is_object, std::vector<std::pair<cell_t, int>>& local_raster_array);

	std::string robot_id_;
	std::array<double,4> a_;
	int16_t separation_;
	std::array<int16_t,2> size_;
	pose_t pose_;
	double weight_;
	int noiseObs_;
	int16_t numParticles_;
	int best_;
	std::vector<std::shared_ptr<Particle> > particles_;
	int sample_std_;
};

}