#define _USE_MATH_DEFINES

#include <iostream>
#include <bitset>
#include <cmath>
#include "slam_utility.h"
#include "robot_simulation.h"
#include "MQTT.h"
#include "log.h"
#include "slam_message.h"

using NTNU::application::SLAM::utility::get_random;

auto first = true;

namespace NTNU::application::SLAM
{

robot_simulation::robot_simulation(const struct robot_simulation_config& config, 
									boost::fibers::buffered_channel<NTNU::application::SLAM::message>& simulator_ch) :
	mqtt_(config.broker_address),
	topic_(config.publish_topic),
	std_dev_(config.std_dev),
	simulator_ch_(simulator_ch),
	obs_x_(),
	obs_y_(),
	init_done_(false),
	obs_x2_(-500),
	obs_y2_(1000),
	obs_y3_(1000),
	obs_x3_(1000),
	obs_x4_(-500),
	obs_y4_(-500),
	x_(),
	y_(),
	x_false_(0),
	y_false_(0),
	theta_(0),
	cntr_(0),
	map_(obstructable_grid(config.grid_size[0],config.grid_size[1])),
	static_noise_(0.0),
	size_(std::array<int16_t, 3>{config.grid_size[0], config.grid_size[1], config.grid_size[2]})
{

	mqtt_.connect();

	//Construct walls surrounding grid

	std::vector<std::pair<int, int>> limit_walls;

	std::pair<int, int> wall_bottom_begin = { 20,20 };		//row, col
	std::pair<int, int> wall_bottom_end = { 20, size_[1]-20 };
	std::pair<int, int> wall_left_begin = { 20, 20 };
	std::pair<int, int> wall_left_end = { size_[0]-20 , 20 };
	std::pair<int, int> wall_right_begin = { 20, size_[1]-20 };
	std::pair<int, int> wall_right_end = { size_[0]-20, size_[1] -20};
	std::pair<int, int> wall_top_begin = { size_[0]-20, 20 };
	std::pair<int, int> wall_top_end = {size_[0]-20, size_[1]-20 };

	limit_walls.push_back(wall_bottom_begin);
	limit_walls.push_back(wall_bottom_end);
	limit_walls.push_back(wall_left_begin);
	limit_walls.push_back(wall_left_end);
	limit_walls.push_back(wall_right_begin);
	limit_walls.push_back(wall_right_end);
	limit_walls.push_back(wall_top_begin);
	limit_walls.push_back(wall_top_end);

	//Obstruct the surrounding walls in the grid
	for (auto i = 0; i < limit_walls.size(); i = i + 2) {
		auto [length, wall] = NTNU::application::SLAM::utility::get_line_between_pts(limit_walls[i], limit_walls[i+1]);

		for (auto j = 0; j < length; j++)
			map_.obstruct(wall[j].first, wall[j].second);
	}

	//Construct other walls, repeat procedure

	std::vector<std::pair<int, int>> walls;

	//Find a nicer looking way to do this... Or "hide" it in another file

	std::pair<int,int> wall_1_begin = {250,1};
	std::pair<int,int> wall_1_end = {250,50};
	std::pair<int, int> wall_2_begin = { 250,50 };
	std::pair<int, int> wall_2_end = { 450,50 };
	std::pair<int, int> wall_3_begin = { 450,50 };
	std::pair<int, int> wall_3_end = { 450,1 };
	std::pair<int, int> wall_4_begin = { 250,100 };
	std::pair<int, int> wall_4_end = { 450,100 };
	std::pair<int, int> wall_5_begin = { 50,50 };
	std::pair<int, int> wall_5_end = { 150,50 };
	std::pair<int, int> wall_6_begin = { 150,50 };
	std::pair<int, int> wall_6_end = { 150, 100 };
	std::pair<int, int> wall_7_begin = { 100,50 };
	std::pair<int, int> wall_7_end = { 100,100 };
	std::pair<int, int> wall_8_begin = { 50,100 };
	std::pair<int, int> wall_8_end = { 200,100 };
	std::pair<int, int> wall_9_begin = { 250,100 };
	std::pair<int, int> wall_9_end = { 250,150 };
	std::pair<int, int> wall_10_begin = { 300,100 };
	std::pair<int, int> wall_10_end = { 300,150 };
	std::pair<int, int> wall_11_begin = { 400,100 };
	std::pair<int, int> wall_11_end = { 400,150 };
	std::pair<int, int> wall_12_begin = { 450,100 };
	std::pair<int, int> wall_12_end = { 450,150 };
	std::pair<int, int> wall_13_begin = { 50,150 };
	std::pair<int, int> wall_13_end = { 150,150 };
	std::pair<int, int> wall_14_begin = { 200,150 };
	std::pair<int, int> wall_14_end = { 300,150 };
	std::pair<int, int> wall_15_begin = { 400,150 };
	std::pair<int, int> wall_15_end = { 450,150 };
	std::pair<int, int> wall_16_begin = { 50,150 };
	std::pair<int, int> wall_16_end = { 50,250 };
	std::pair<int, int> wall_17_begin = { 150,150 };
	std::pair<int, int> wall_17_end = { 150,250 };
	std::pair<int, int> wall_18_begin = { 350,150 };
	std::pair<int, int> wall_18_end = { 350,250 };
	std::pair<int, int> wall_19_begin = { 200,200 };
	std::pair<int, int> wall_19_end = { 300,200 };
	std::pair<int, int> wall_20_begin = { 200,200 };
	std::pair<int, int> wall_20_end = { 200,300 };
	std::pair<int, int> wall_21_begin = { 300,200 };
	std::pair<int, int> wall_21_end = { 300,300 };;
	std::pair<int, int> wall_22_begin = { 50,250 };
	std::pair<int, int> wall_22_end = { 150,250 };
	std::pair<int, int> wall_23_begin = { 350,250 };
	std::pair<int, int> wall_23_end = { 400,250 };
	std::pair<int, int> wall_24_begin = { 450,250 };
	std::pair<int, int> wall_24_end = { 500,250 };
	std::pair<int, int> wall_25_begin = { 400,250 };
	std::pair<int, int> wall_25_end = { 400,300 };
	std::pair<int, int> wall_26_begin = { 450,250 };
	std::pair<int, int> wall_26_end = { 450,300 };
	std::pair<int, int> wall_27_begin = { 150,300 };
	std::pair<int, int> wall_27_end = { 200,300 };
	std::pair<int, int> wall_28_begin = { 250,300 };
	std::pair<int, int> wall_28_end = { 300,300 };
	std::pair<int, int> wall_29_begin = { 350,300 };
	std::pair<int, int> wall_29_end = { 400,300 };
	std::pair<int, int> wall_30_begin = { 450,300 };
	std::pair<int, int> wall_30_end = { 500,300 };
	std::pair<int, int> wall_31_begin = { 150,300 };
	std::pair<int, int> wall_31_end = { 150,350 };
	std::pair<int, int> wall_32_begin = { 350,300 };
	std::pair<int, int> wall_32_end = { 350,350 };
	std::pair<int, int> wall_33_begin = { 150,350 };
	std::pair<int, int> wall_33_end = { 350,350 };
	std::pair<int, int> wall_34_begin = { 150,400 };
	std::pair<int, int> wall_34_end = { 200,400 };
	std::pair<int, int> wall_35_begin = { 300,400 };
	std::pair<int, int> wall_35_end = { 350,400 };
	std::pair<int, int> wall_36_begin = { 450,400 };
	std::pair<int, int> wall_36_end = { 500,400 };
	std::pair<int, int> wall_37_begin = { 150,400 };
	std::pair<int, int> wall_37_end = { 150,500 };
	std::pair<int, int> wall_38_begin = { 250,400 };
	std::pair<int, int> wall_38_end = { 250,500 };
	std::pair<int, int> wall_39_begin = { 350,400 };
	std::pair<int, int> wall_39_end = { 350,500 };
	std::pair<int, int> wall_40_begin = { 400,400 };
	std::pair<int, int> wall_40_end = { 400,500 };
	std::pair<int, int> wall_41_begin = { 200,450 };
	std::pair<int, int> wall_41_end = { 250,450 };
	std::pair<int, int> wall_42_begin = { 200,450 };
	std::pair<int, int> wall_42_end = { 200,500 };
	std::pair<int, int> wall_43_begin = { 275,225 };
	std::pair<int, int> wall_43_end = { 275,300 };
	std::pair<int, int> wall_44_begin = { 200,225 };
	std::pair<int, int> wall_44_end = { 275,225 };

	walls.push_back(wall_1_begin);
	walls.push_back(wall_1_end);
	walls.push_back(wall_2_begin);
	walls.push_back(wall_2_end);
	walls.push_back(wall_3_begin);
	walls.push_back(wall_3_end);
	walls.push_back(wall_4_begin);
	walls.push_back(wall_4_end);
	walls.push_back(wall_5_begin);
	walls.push_back(wall_5_end);
	walls.push_back(wall_6_begin);
	walls.push_back(wall_6_end);
	walls.push_back(wall_7_begin);
	walls.push_back(wall_7_end);
	walls.push_back(wall_8_begin);
	walls.push_back(wall_8_end);
	walls.push_back(wall_9_begin);
	walls.push_back(wall_9_end);
	walls.push_back(wall_10_begin);
	walls.push_back(wall_10_end);
	walls.push_back(wall_11_begin);
	walls.push_back(wall_11_end);
	walls.push_back(wall_12_begin);
	walls.push_back(wall_12_end);
	walls.push_back(wall_13_begin);
	walls.push_back(wall_13_end);
	walls.push_back(wall_14_begin);
	walls.push_back(wall_14_end);
	walls.push_back(wall_15_begin);
	walls.push_back(wall_15_end);
	walls.push_back(wall_16_begin);
	walls.push_back(wall_16_end);
	walls.push_back(wall_17_begin);
	walls.push_back(wall_17_end);
	walls.push_back(wall_18_begin);
	walls.push_back(wall_18_end);
	walls.push_back(wall_19_begin);
	walls.push_back(wall_19_end);
	walls.push_back(wall_20_begin);
	walls.push_back(wall_20_end);
	walls.push_back(wall_21_begin);
	walls.push_back(wall_21_end);
	walls.push_back(wall_22_begin);
	walls.push_back(wall_22_end);
	walls.push_back(wall_23_begin);
	walls.push_back(wall_23_end);
	walls.push_back(wall_24_begin);
	walls.push_back(wall_24_end);
	walls.push_back(wall_25_begin);
	walls.push_back(wall_25_end);
	walls.push_back(wall_26_begin);
	walls.push_back(wall_26_end);
	walls.push_back(wall_27_begin);
	walls.push_back(wall_27_end);
	walls.push_back(wall_28_begin);
	walls.push_back(wall_28_end);
	walls.push_back(wall_29_begin);
	walls.push_back(wall_29_end);
	walls.push_back(wall_30_begin);
	walls.push_back(wall_30_end);
	walls.push_back(wall_31_begin);
	walls.push_back(wall_31_end);
	walls.push_back(wall_32_begin);
	walls.push_back(wall_32_end);
	walls.push_back(wall_33_begin);
	walls.push_back(wall_33_end);
	walls.push_back(wall_34_begin);
	walls.push_back(wall_34_end);
	walls.push_back(wall_35_begin);
	walls.push_back(wall_35_end);
	walls.push_back(wall_36_begin);
	walls.push_back(wall_36_end);
	walls.push_back(wall_37_begin);
	walls.push_back(wall_37_end);
	walls.push_back(wall_38_begin);
	walls.push_back(wall_38_end);
	walls.push_back(wall_39_begin);
	walls.push_back(wall_39_end);
	walls.push_back(wall_40_begin);
	walls.push_back(wall_40_end);
	walls.push_back(wall_41_begin);
	walls.push_back(wall_41_end);
	walls.push_back(wall_42_begin);
	walls.push_back(wall_42_end);
	walls.push_back(wall_43_begin);
	walls.push_back(wall_43_end);
	walls.push_back(wall_44_begin);
	walls.push_back(wall_44_end);

	//Localisation help-walls

	
	std::pair<int, int> help_1_begin = { 250,300 };
	std::pair<int, int> help_1_end = { 250,320 };
	std::pair<int, int> help_2_begin = { 315,250 };
	std::pair<int, int> help_2_end = { 275,250 };
	std::pair<int, int> help_3_begin = { 250, 190 };
	std::pair<int, int> help_3_end = { 250,220 };
	std::pair<int, int> help_4_begin = { 200,250 };
	std::pair<int, int> help_4_end = { 190,250 };

	std::pair<int, int> help_5_begin = { 400, 60 };
	std::pair<int, int> help_5_end = { 400, 40 };
	std::pair<int, int> help_6_begin = { 365,200 };
	std::pair<int, int> help_6_end = { 350,200 };
	std::pair<int, int> help_7_begin = { 350, 60 };
	std::pair<int, int> help_7_end = { 350,40 };
	std::pair<int, int> help_8_begin = { 300,60 };
	std::pair<int, int> help_8_end = { 300 ,40 };

	std::pair<int, int> help_9_begin = { 100,120 };
	std::pair<int, int> help_9_end = { 100, 100 };
	std::pair<int, int> help_10_begin = { 200,120 };
	std::pair<int, int> help_10_end = { 200,100 };
	std::pair<int, int> help_11_begin = { 250, 190 };
	std::pair<int, int> help_11_end = { 250,220 };
	std::pair<int, int> help_12_begin = { 350,150 };
	std::pair<int, int> help_12_end = { 340,150 };

	std::pair<int, int> help_13_begin = { 400, 60 };
	std::pair<int, int> help_13_end = { 400, 40 };
	std::pair<int, int> help_14_begin = { 40 ,300 };
	std::pair<int, int> help_14_end = { 20, 300 };
	std::pair<int, int> help_15_begin = { 40, 350 };
	std::pair<int, int> help_15_end = { 20 , 350 };
	std::pair<int, int> help_16_begin = { 40, 400 };
	std::pair<int, int> help_16_end = { 20 , 400 };

	std::pair<int, int> help_17_begin = { 100, 260 };
	std::pair<int, int> help_17_end = { 100 , 240 };
	std::pair<int, int> help_18_begin = { 365, 330 };
	std::pair<int, int> help_18_end = { 350 , 330 };
	std::pair<int, int> help_19_begin = { 490, 270 };
	std::pair<int, int> help_19_end = { 470 , 270 };
	std::pair<int, int> help_20_begin = { 380, 300 };
	std::pair<int, int> help_20_end = { 380 , 315 };
	std::pair<int, int> help_21_begin = { 125, 50 };
	std::pair<int, int> help_21_end = { 125 , 60 };
	std::pair<int, int> help_22_begin = { 75, 50 };
	std::pair<int, int> help_22_end = { 75 , 60 };

	walls.push_back(help_1_begin);
	walls.push_back(help_1_end);
	walls.push_back(help_2_begin);
	walls.push_back(help_2_end);
	walls.push_back(help_3_begin);
	walls.push_back(help_3_end);
	walls.push_back(help_4_begin);
	walls.push_back(help_4_end);
	walls.push_back(help_5_begin);
	walls.push_back(help_5_end);
	walls.push_back(help_6_begin);
	walls.push_back(help_6_end);
	walls.push_back(help_7_begin);
	walls.push_back(help_7_end);
	walls.push_back(help_8_begin);
	walls.push_back(help_8_end);
	walls.push_back(help_9_begin);
	walls.push_back(help_9_end);
	walls.push_back(help_10_begin);
	walls.push_back(help_10_end);
	walls.push_back(help_11_begin);
	walls.push_back(help_11_end);
	walls.push_back(help_12_begin);
	walls.push_back(help_12_end);
	walls.push_back(help_13_begin);
	walls.push_back(help_13_end);
	walls.push_back(help_14_begin);
	walls.push_back(help_14_end);
	walls.push_back(help_15_begin);
	walls.push_back(help_15_end);
	walls.push_back(help_16_begin);
	walls.push_back(help_16_end);
	walls.push_back(help_17_begin);
	walls.push_back(help_17_end);
	walls.push_back(help_18_begin);
	walls.push_back(help_18_end);
	walls.push_back(help_19_begin);
	walls.push_back(help_19_end);
	walls.push_back(help_20_begin);
	walls.push_back(help_20_end);
	walls.push_back(help_21_begin);
	walls.push_back(help_21_end);
	walls.push_back(help_22_begin);
	walls.push_back(help_22_end);
	

	for (auto i = 0; i < walls.size(); i = i + 2) {
		auto [length, wall] = NTNU::application::SLAM::utility::get_line_between_pts(walls[i], walls[i+1]);

		for (auto j = 0; j < length; j++) {
			map_.obstruct(wall[j].first, wall[j].second);
		}
	}

	

}

bool robot_simulation::new_run()
{
	if (!init_done_) {

		//First send message to state presence.
		NTNU::application::SLAM::message first_msg(topic_);
		first_msg.set_scan();

		if (mqtt_.publish(topic_, first_msg.serialize()) != networking::protocols::MQTT::MQTT::SUCCESS) {
			LOG_WARN("Simulation bad publish?");
			//std::cout << "Simulation bad publish?\n";
			return false;
		}

		NTNU::application::SLAM::message sim_msg;

		auto attempts = 0;
		auto max_attempts = 1200;

		//Wait for init_pose
		while (simulator_ch_.try_pop(sim_msg) !=
			boost::fibers::channel_op_status::success) {

			if (attempts == max_attempts) {
				//std::cout << "Simulator received no response from server" << std::endl;
				LOG_WARN("Simulator received no response from server");
				return false;
			}
			attempts++;
			boost::this_thread::sleep_for(boost::chrono::milliseconds{ 100 });
		}

		auto [init_x, init_y, init_theta] = sim_msg.robot_pos();

		x_ = init_x;
		y_ = init_y;
		theta_ = init_theta;
		LOG_INFO("Init set to : x: {:>10}, y: {:>10}, theta: {:>10}", x_, y_, theta_);
		          
		//std::cout << "Init set to: " << x_ << ", " << y_ << ", " << theta_ << std::endl;

		//Send init pos as target, so that the simulator sends scan from init pos
		NTNU::application::SLAM::message init_target_msg("v2/server/simulated/cmd");

		NTNU::application::SLAM::message::position pos{ init_x, init_y };
		init_target_msg.set_payload(pos, true);

		auto result = simulator_ch_.push(init_target_msg);
		if (result != boost::fibers::channel_op_status::success) {
			//std::cerr << "Init target msg did not succeed!\n";
			LOG_ERROR("Init target message did not succeed!");
		}

		init_done_ = true;

	}
	else {
		NTNU::application::SLAM::message target_msg;
		//Check if new target has arrived
		if (simulator_ch_.try_pop(target_msg) ==
			boost::fibers::channel_op_status::success) {
			auto collision = false;

			NTNU::application::SLAM::message::position odom = {0,0};
			double odom_angle = 0.0;

			if (target_msg.type() == NTNU::application::SLAM::message::msg_type_in::SIM_TARGET) {

				auto target = target_msg.target();

				auto angle = atan2((target.y - y_), (target.x - x_)) * 180 / M_PI;
				       
				LOG_INFO("New target  : x: {:>10}, y: {:>10}, theta: {:>10}", target.x, target.y, angle);
				//std::cout << "New target: " << target.x << ", " << target.y << ", Gives theta: " << angle << std::endl;

				//Simulate 100 mm collision detection
				auto extended_target = std::pair<int, int>({ target.x + 100 * cos(angle), target.y + 100 * sin(angle) });

				auto pos_grid = NTNU::application::SLAM::utility::coord_to_row_col(std::array<int16_t, 2>{size_[0], size_[1]}, size_[2], x_, y_);
				//auto target_grid = NTNU::application::SLAM::utility::coord_to_row_col(std::array<int, 2>{size_[0], size_[1]}, size_[2], target_x, target_y);
				auto target_grid = NTNU::application::SLAM::utility::coord_to_row_col(std::array<int16_t, 2>{size_[0], size_[1]}, size_[2], extended_target.first, extended_target.second);

				auto driven_distance = 0;

				if (pos_grid && target_grid) {
					auto [pos_row, pos_col] = pos_grid.value();
					auto [target_row, target_col] = target_grid.value();

					auto [distance, path] = NTNU::application::SLAM::utility::get_line_between_pts(std::make_pair(pos_row, pos_col), std::make_pair(target_row, target_col));
					driven_distance = distance * 20;
					//Check line if it hits an obstructed grid cell
					//This simulates obstacle detection in the robot
					for (auto k = 0; k < distance; k++) {

						if (map_.is_obstructed(path[k].first, path[k].second)) {
							//It is hits, convert to coords and set as observation
							//td::cout << "Collision" << std::endl;
							collision = true;

							//Don't move if obstruction is closer than 100 mm
							//(5 * 20 mm cells = 100 mm)
							if (k <= 5) 
								break;

							//std::cout << "But move" << std::endl;

							auto obs_point = NTNU::application::SLAM::utility::row_col_to_coord(std::array<int16_t, 2>{size_[0], size_[1]}, size_[2], path[k - 5].first, path[k - 5].second);
							//std::cout << "Found obstructed cell" << std::endl;
							if (obs_point) {
								auto [point_x, point_y] = obs_point.value();


								odom.x = point_x - x_;
								odom.y = point_y - y_;
								odom_angle = angle - theta_;

								//calculate the movements between previous and present instances
								auto rot_1 = atan2(odom.y, odom.x) - theta_ * M_PI / 180;
								auto trans = sqrt(pow(odom.x, 2) + pow(odom.y, 2));
								auto rot_2 = odom_angle * M_PI / 180 - rot_1;

								//calculate correct noise stds according to the motion model
								auto rot_1_std = sqrt(0.001 * abs(rot_1) + 0.00001 * abs(trans));
								auto trans_std = sqrt(0.0001 * abs(trans) + 0.001 * (abs(rot_1) + abs(rot_2)));
								auto rot_2_std = sqrt(0.001 * abs(rot_2) + 0.00001 * abs(trans));

								//add the gaussian distributed noise
								auto rot_1_sampled = get_random(rot_1, rot_1_std);
								auto trans_sampled = get_random(trans, trans_std);
								auto rot_2_sampled = get_random(rot_2, rot_2_std);

								//update the pose
								x_ = trans_sampled * cos(theta_ * M_PI / 180.0 + rot_1_sampled) + x_;
								y_ = trans_sampled * sin(theta_ * M_PI / 180.0 + rot_1_sampled) + y_;
								theta_ = rot_1_sampled * 180.0 / M_PI + rot_2_sampled * 180.0 / M_PI + theta_;

								/* Send exact pose */
								/*
								x_ = x_ + odom.x;
								y_ = y_ + odom.y;
								theta_ = theta_ + odom_angle;
								*/

								LOG_INFO("Real pos    : x: {:>10}, y: {:>10}, theta: {:>10}", x_, y_, theta_);
							}

							break;
						}
					}

					if (!collision) {
						if (driven_distance > 20) {
							odom.x = target.x - x_;
							odom.y = target.y - y_;
							odom_angle = angle - theta_;

							//calculate the movements between previous and present instances
							auto rot_1 = atan2(odom.y, odom.x) - theta_ * M_PI / 180;
							auto trans = sqrt(pow(odom.x, 2) + pow(odom.y, 2));
							auto rot_2 = odom_angle * M_PI / 180 - rot_1;

							//calculate correct noise stds according to the motion model
							auto rot_1_std = sqrt(0.001 * abs(rot_1) + 0.00001 * trans);
							auto trans_std = sqrt(0.0001 * trans + 0.001 * (abs(rot_1) + abs(rot_2)));
							auto rot_2_std = sqrt(0.001 * abs(rot_2) + 0.00001 * trans);

							//add the gaussian distributed noise
							auto rot_1_sampled = get_random(rot_1, rot_1_std);
							auto trans_sampled = get_random(trans, trans_std);
							auto rot_2_sampled = get_random(rot_2, rot_2_std);

							//update the pose
							x_ = trans_sampled * cos(theta_ * M_PI / 180.0 + rot_1_sampled) + x_;
							y_ = trans_sampled * sin(theta_ * M_PI / 180.0 + rot_1_sampled) + y_;
							theta_ = rot_1_sampled * 180.0 / M_PI + rot_2_sampled * 180.0 / M_PI + theta_;

							/* Send exact pose*/
							/*
							x_ = x_ + odom.x;
							y_ = y_ + odom.y;
							theta_ = theta_ + odom_angle;
							*/
						}
						LOG_INFO("Real pos    : x: {:>10}, y: {:>10}, theta: {:>10}", x_, y_, theta_);

					}
				}
			}
			else
				return true;

			auto max_range = 1000.0;

			auto border_x = size_[1] * size_[2] / 2;
			auto border_y = size_[0] * size_[2] / 2;

			for (auto i = 0; i < 90; i++) {
				obs_x_.clear();
				obs_y_.clear();

				double obs_bit = 0.0;

				for (auto j = 0; j < 4; j++) {

					//Calculate point at max range, in global coords
					auto obs_x = x_ + max_range * cos((i + theta_ + (j * 90)) * M_PI / 180);
					auto obs_y = y_ + max_range * sin((i + theta_ + (j * 90)) * M_PI / 180);



					//Make sure the observations are within the boudaries
					if (abs(obs_x) > border_x) {
						if (obs_x < border_x)
							obs_x = -border_x;
						else
							obs_x = border_x - 1;
					}

					if (abs(obs_y) > border_y) {
						if (obs_y < border_y)
							obs_y = -border_y;
						else
							obs_y = border_y - 1;
					}

					//std::cout << "Max obs: " << obs_x << ", " << obs_y << std::endl;

					auto pos_grid = NTNU::application::SLAM::utility::coord_to_row_col(std::array<int16_t, 2>{size_[0], size_[1]}, size_[2], x_, y_);
					auto obs_grid = NTNU::application::SLAM::utility::coord_to_row_col(std::array<int16_t, 2>{size_[0], size_[1]}, size_[2], obs_x, obs_y);

					if (pos_grid && obs_grid) {
						auto [pos_row, pos_col] = pos_grid.value();
						auto [obs_row, obs_col] = obs_grid.value();

						//std::cout << "Pos grid: " << pos_row << ", " << pos_col << std::endl;
						//std::cout << "Max obs grid: " << obs_row << ", " << obs_col << std::endl;

						auto [range, laser] = NTNU::application::SLAM::utility::get_line_between_pts(std::make_pair(pos_row, pos_col), std::make_pair(obs_row, obs_col));

						auto nothing_in_range = true;

						//Check line if it hits an obstructed grid cell
						for (auto k = 0; k < range; k++) {
							//std::cout << "Checking line cell: " << laser[k].first << ", " << laser[k].second << std::endl;
							if (map_.is_obstructed(laser[k].first, laser[k].second)) {
								//It is hits, convert back to coords and set as observation
								auto obs_point = NTNU::application::SLAM::utility::row_col_to_coord(std::array<int16_t, 2>{size_[0], size_[1]}, size_[2], laser[k].first, laser[k].second);
								//std::cout << "Found obstructed cell" << std::endl;
								if (obs_point) {
									auto [point_x, point_y] = obs_point.value();
									//Transform to local coords
									obs_x_.push_back((point_x - x_) * cos(-theta_ * M_PI / 180) - (point_y - y_) * sin(-theta_ * M_PI / 180));
									obs_y_.push_back((point_x - x_) * sin(-theta_ * M_PI / 180) + (point_y - y_) * cos(-theta_ * M_PI / 180));
								}

								//Set correct bit
								obs_bit += pow(2, (3 - j));
								nothing_in_range = false;

								//std::cout << "Set bit: " << obs_bit << std::endl;

								// No need to search rest of line
								break;
							}
						}

						if (nothing_in_range) {
							//Send as local coords
							obs_x_.push_back(max_range * cos((i + (j * 90)) * M_PI / 180));
							obs_y_.push_back(max_range * sin((i + (j * 90)) * M_PI / 180));
						}
					}
				}

				NTNU::application::SLAM::message msg(topic_);
				NTNU::application::SLAM::message::pose pos{ odom.x, odom.y , odom_angle };
				NTNU::application::SLAM::message::position obs{ obs_x_[0], obs_y_[0] };
				NTNU::application::SLAM::message::position obs2{ obs_x_[1], obs_y_[1] };
				NTNU::application::SLAM::message::position obs3{ obs_x_[2], obs_y_[2] };
				NTNU::application::SLAM::message::position obs4{ obs_x_[3], obs_y_[3] };;

				int8_t bits_int;

				bits_int = (int8_t)(obs_bit);

				msg.set_payload(pos, { obs, obs2, obs3, obs4 }, bits_int);

				if (mqtt_.publish(topic_, msg.serialize()) != networking::protocols::MQTT::MQTT::SUCCESS) {
					LOG_WARN("Simulation bad publish?");
					//std::cout << "Simulation bad publish?\n";
				}

			}

			//Send scan_border message
			NTNU::application::SLAM::message scan_msg(topic_);
			scan_msg.set_scan();

			first = false;

			if (mqtt_.publish(topic_, scan_msg.serialize()) != networking::protocols::MQTT::MQTT::SUCCESS) {
				LOG_WARN("Simulation bad publish?");
				//std::cout << "Simulation bad publish?\n";
			}

			if (collision)
			{
				NTNU::application::SLAM::message collision_msg(topic_);
				collision_msg.set_collision();
				if (mqtt_.publish(topic_, collision_msg.serialize()) != networking::protocols::MQTT::MQTT::SUCCESS) {
					LOG_WARN("Simulation bad publish?");
					//std::cout << "Simulation bad publish?\n";
				}
			}
		}
	}
	return true;
}


}
