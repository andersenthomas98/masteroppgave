#pragma once

#include <boost/fiber/buffered_channel.hpp>
#include <string>
#include "particle.h"
#include "MQTT.h"
#include "obstructable_grid.h"

namespace NTNU::application::SLAM
{

struct robot_simulation_config {
	std::string broker_address;
	std::string publish_topic;
	int16_t std_dev; // How much the simulated robot can move between each call to run
	std::array<int16_t, 3> grid_size; //and separation
};

class robot_simulation
{
public:
	robot_simulation(const struct robot_simulation_config& config, boost::fibers::buffered_channel<NTNU::application::SLAM::message>& simulator_ch);

	bool new_run();

	~robot_simulation() {};

private:
	NTNU::networking::protocols::MQTT::MQTT mqtt_;
	std::string topic_;
	boost::fibers::buffered_channel<NTNU::application::SLAM::message>& simulator_ch_;
	bool init_done_;

	int16_t x_;
	int16_t y_;
	int16_t theta_;
	std::vector<int> obs_x_;
	std::vector<int> obs_y_;
	int16_t x_false_;
	int16_t y_false_;
	int16_t obs_x2_;
	int16_t obs_x3_;
	int16_t obs_x4_;
	int16_t obs_y2_;
	int16_t obs_y3_;
	int16_t obs_y4_;
	int16_t cntr_;
	int16_t std_dev_;
	double static_noise_;
	obstructable_grid map_;
	std::array <int16_t, 3> size_;
};

}
