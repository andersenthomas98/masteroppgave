#pragma once

#include <SFML/Graphics.hpp>
#include <map>
#include "slam_grid.h"
#include "panel.h"
#include "squares.h"
#include "updatable.h"
#include "callbacks.h"
#include "robot.h"

namespace NTNU::application::SLAM
{

class message;

enum class robots_events
{
	ROBOT_MOVED,				// Context: Tuple (id, x, y)
	ROBOT_FOUND_OBSTACLE,		// Context: Tuple (id, x, y, map)
	ROBOT_CLEARED_POINTS,		// Context: std::vector<pair<int, int>> of all obstacles being cleared
	ROBOT_IDLE,
	ROBOT_NEW
};

class robots :
	public sf::Drawable,
	public sf::Transformable,
	public NTNU::gui::panel::panel,
	public NTNU::gui::base::updatable,
	public NTNU::utility::callbacks<robots_events>
{
public:
	robots( NTNU::application::SLAM::slam_grid* grid);
	~robots() {};

	void feed_message(const message& msg);

	const std::unique_ptr<robot>* create_robot(const std::string& robot_id, pose_t init_pose, std::array<int16_t, 3> size);

	std::vector<std::pair<int16_t, int16_t>> get_all_obstacles() const;
	std::vector<std::string> get_all_robot_ids() const;

	void update_path_for_robot(const std::string& robot, std::pair<int, int> target);

	std::optional<std::pair<int16_t, int16_t>> get_target(const std::string& robot_id) const;
	std::optional<std::pair<int16_t, int16_t>> get_next_point(const std::string& robot_id) const;
	std::optional<std::pair<double, double>> position(const std::string& robot_id) const;

	void update(std::string robot_id, pose_t pose, std::vector<message::position> obs, std::vector<bool> is_object);
	void update(std::string robot_id);
	void update(sf::Time delta) override;
	
	bool navigate_obstacle() const;
	bool robot_navigate();
	bool search_grid();
	std::array<int16_t, 3> get_grid_dim();
	std::vector<std::pair<float, float>> get_square_values(float x, float y);
	

	void set_path(const std::string& robot_id, const std::vector<std::pair<float, float>> coords);
	void remove_old_point(const std::string& robot_id);
	void clear_path(const std::string& robot_id);

	void draw(sf::RenderTarget & target, sf::RenderStates states) const override;

private:
	const std::unique_ptr<robot>* get_robot(const std::string& robot_id) const;

	NTNU::application::SLAM::slam_grid& grid_;

	std::map<std::string, std::unique_ptr<robot>> robots_;

	bool robots_navigate_obstacle_;
	bool robots_navigate_;
	bool robots_search_;
	int8_t counter;
};

}

