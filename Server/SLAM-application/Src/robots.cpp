#include "robots.h"
#include "robot.h"
#include "slam_message.h"
#include "log.h"
#include "slam_utility.h"
#include "grid_path_solver.h"
#include "imgui.h"
#include <boost/fiber/all.hpp>
#include <iostream>

using boost::this_fiber::yield;

namespace NTNU::application::SLAM
{

	robots::robots(NTNU::application::SLAM::slam_grid* grid) :
		robots_navigate_obstacle_(false),
		robots_navigate_(false),
		robots_search_(false),
		counter(0),
		grid_(*grid)
{
	
}

void robots::feed_message(const message& msg)
{
	auto robot_id = msg.sender();
	auto robot = get_robot(robot_id);
	if (!robot) {
		call_callback(robots_events::ROBOT_NEW, robot_id);
	} else {

		if (msg.type() == message::msg_type_in::UPDATE) {
			auto [msg_x, msg_y, msg_theta] = msg.robot_pos();
			pose_t new_pose = { msg_x , msg_y , msg_theta };
			update(robot_id, new_pose, msg.obstacles(), msg.is_object());
		} 
		else if (msg.type() == message::msg_type_in::SCAN_BORDER) {
			update(robot_id);
		}
		else if (msg.type() == message::msg_type_in::COL_DET) {
			auto target = get_target(robot_id);

			if (target)
				if (auto result = NTNU::application::SLAM::utility::coord_to_row_col({ grid_.rows() , grid_.cols() }, grid_.separation(), target.value().first, target.value().second); result)
					update_path_for_robot(robot_id, result.value());
		} 
		else if (msg.type() == message::msg_type_in::LINE) {
			LOG_INFO("Received line in robot.cpp");
			auto [msg_x, msg_y, msg_theta] = msg.robot_pos();
			pose_t new_pose = { msg_x , msg_y , msg_theta };
			auto new_line = msg.get_line();
			update(robot_id, new_pose, new_line);
			LOG_INFO("({},{}) --- ({},{})", new_line.startPoint.x, new_line.startPoint.y, new_line.endPoint.x, new_line.endPoint.y);

		}

		set_fun([&]() {
			const ImVec2 btn_size{ 200, 0 };
			if (robots_.size() == 0)
				return;

			ImGui::Checkbox("Navigate Around Obstacles", &robots_navigate_obstacle_);
			ImGui::Checkbox("Search Grid", &robots_search_);

			if (ImGui::Button("Start navigation in a square", btn_size)) {
				robots_navigate_ = true;

			}
			if (ImGui::Button("Stop navigation", btn_size)) {
				robots_navigate_ = false;

			}
			if (ImGui::BeginTabBar("Robots Tab Bar", ImGuiTabBarFlags_None))
			{
				for (const auto& entry : robots_)
				{
					const auto& [id, robot] = entry;
					if (ImGui::BeginTabItem(id.c_str()))
					{
						robot->show();

						ImGui::EndTabItem();
					}
				}

				ImGui::EndTabBar();

			}

		});
	}
}

void robots::update_path_for_robot(const std::string& robot, std::pair<int, int> target) {
	//std::cout << "update_path_for_robot: " << robot << ",  target: " << target.first << " , " << target.second << "\n";
	auto get_pos = position(robot);
	if (!get_pos)
		return;

	auto source = get_pos.value();	// robot position
	//std::cout << "Robot in real world: " << source.first << " , " << source.second << "\n";
	// Robot positions are given in global coordinates...
	//std::cout << "update_path_for_robot rows,cols: " << rows << cols << std::endl;
	auto result = NTNU::application::SLAM::utility::coord_to_row_col({ grid_.rows() , grid_.cols() }, grid_.separation(), source.first, source.second);
	if (!result)
		return;

	auto [robo_row, robo_col] = result.value();
	source = { robo_row, robo_col };
	//std::cout << "Robot at map: " << robo_row << " , " << robo_col << "\n";

	//auto path = std::optional<std::vector<NTNU::graph::grid::point>>();
	auto path = std::vector<NTNU::graph::grid::point>();

	//Bypass solving path bco slow bfs
	path.push_back(result.value());
	path.push_back(target);

	/* Bypass solving path bco slow bfs */
	/*
	while (!path) {
		// ... paths are solved in terms of the underlying grid (which uses rows and columns)...
		path = NTNU::graph::pathfinding::solve(grid_.get_filtered_grid(), result.value(), target);

		//If no path is found, robot probably believes it is trapped.
		//Choose random position close to belief position.
		if (!path) {
			std::cout << "No path" << std::endl;
			source = {NTNU::application::SLAM::utility::get_random(source.first, 0.5), NTNU::application::SLAM::utility::get_random(source.first, 1) };
		}
			
	}
	*/
	// ... reduce the given path to straight lines ...
	//auto rpath = NTNU::graph::pathfinding::reduce(path.value());

	// ... and convert it back to global coordinates ...
	//if (auto coords = NTNU::application::SLAM::utility::points_to_coords(grid_, rpath); navigate_obstacle())
	if (auto coords = NTNU::application::SLAM::utility::points_to_coords(grid_, path); navigate_obstacle())
	{
		// ... apply this new path to the given robot.
		clear_path(robot);
		set_path(robot, coords);
	}
	else
	{
		set_path(robot, { coords.front(), coords.back() });
	}
}

std::vector<std::pair<int16_t, int16_t>> robots::get_all_obstacles() const
{
	std::vector<std::pair<int16_t, int16_t>> obstacles;
	for (const auto& entry : robots_)
	{
		const auto&[_, robot] = entry;
		auto robot_obstacles = robot->get_obstacles();
		std::copy(robot_obstacles.begin(), robot_obstacles.end(), std::back_inserter(obstacles));
	}

	return obstacles;
}

std::vector<std::string> robots::get_all_robot_ids() const
{
	std::vector<std::string> ids;

	for (const auto& entry : robots_)
	{
		const auto&[id, _] = entry;
		ids.push_back(id);
	}

	return ids;
}

std::optional<std::pair<int16_t, int16_t>> robots::get_target(const std::string & robot_id) const
{
	auto robot = get_robot(robot_id);
	if (!robot)
		return std::nullopt;

	return robot->get()->get_target();
}

std::optional<std::pair<int16_t, int16_t>> robots::get_next_point(const std::string & robot_id) const
{
	auto robot = get_robot(robot_id);
	if (!robot)
		return std::nullopt;

	return robot->get()->get_next_on_path();
}

void robots::remove_old_point(const std::string& robot_id)
{
	auto robot = get_robot(robot_id);
	if (!robot)
		return;
	
	robot->get()->remove_old_point();

}

void robots::clear_path(const std::string& robot_id)
{
	auto robot = get_robot(robot_id);
	if (!robot)
		return;
	//std::cout << "Clear path (robots)" << std::endl;
	robot->get()->clear_path();

}

std::optional<std::pair<double, double>> robots::position(const std::string & robot_id) const
{
	auto robot = get_robot(robot_id);
	if (!robot)
		return std::nullopt;

	//Pose is automatically updated by mean of particle poses.
	auto pos = robot->get()->getPosition();
	return std::make_pair((double)pos.x, (double)pos.y);
}

void robots::update(std::string robot_id, pose_t new_pose, std::vector<message::position> obs, std::vector<bool> is_object)
{
	for (const auto& entry : robots_)
	{
		const auto& [_, robot] = entry;
		if (robot->getName() == robot_id)
			robot->update(new_pose, obs, is_object);
	}
}

void robots::update(std::string robot_id) {
	for (const auto& entry : robots_)
	{
		const auto& [_, robot] = entry;
		if (robot->getName() == robot_id)
			robot->update();
	}
}

void robots::update(std::string robot_id, pose_t new_pose, message::line new_line) {
	for (const auto& entry : robots_) {
		const auto& [_, robot] = entry;
		if (robot->getName() == robot_id) {
			robot->update(new_pose, new_line);
		}
		
	}

}

//Not sure why, but this seems to be necessary
void robots::update(sf::Time delta) {

}

bool robots::navigate_obstacle() const
{
	return robots_navigate_obstacle_;
}
bool robots::robot_navigate() {
	return robots_navigate_;
}

bool robots::search_grid() {
	return robots_search_;
}

void robots::set_path(const std::string & robot_id, const std::vector<std::pair<float, float>> coords)
{
	auto robot = get_robot(robot_id);
	if (!robot)
		return;

	robot->get()->set_path(coords);
}

void robots::draw(sf::RenderTarget & target, sf::RenderStates states) const
{
	for (const auto& entry : robots_)
	{
		const auto&[_, robot] = entry;
		robot->draw(target, states);
	}
}

const std::unique_ptr<robot>* robots::create_robot(const std::string& robot_id, pose_t init_pose, std::array<int16_t, 3> size)
{
	auto[robot_entry, robot_success] = robots_.emplace(std::make_pair(robot_id, std::make_unique<robot>(robot_id, init_pose, size)));
	if (!robot_success)
		throw std::exception("Could not emplace robot.");

	auto& robot = robot_entry->second;
	robot->setColor(sf::Color::Blue);

	robot->enable_callback(robot_events::POINTS_CLEARED, [&](std::any) {
		call_callback(robots_events::ROBOT_CLEARED_POINTS, robot->get_obstacles());
	});

	robot->enable_callback(NTNU::application::SLAM::robot_events::OBSTACLE, [&](std::any context) {
	try {
		call_callback(robots_events::ROBOT_FOUND_OBSTACLE, context);
	}
	catch (const std::bad_any_cast & e) { 
		//std::cout << e.what(); 
		LOG_ERROR("{}", e.what());
	}
	});

	return &robot;
}

const std::unique_ptr<robot>* robots::get_robot(const std::string& robot_id) const
{
	if (auto it = robots_.find(robot_id); it != robots_.end())
		return &robots_.find(robot_id)->second;

	return nullptr;
}

std::array<int16_t, 3> robots::get_grid_dim() {
	return std::array<int16_t, 3>({ grid_.rows(), grid_.cols(), grid_.separation() });
}

std::vector<std::pair<float, float>> robots::get_square_values(float x, float y) {
	std::vector<std::pair<float, float>> coords;
	coords.emplace_back(x + 10, y);
	coords.emplace_back(x + 10, y + 10);
	coords.emplace_back(x, y + 10);
	coords.emplace_back(x, y);
	return coords;
}
}
