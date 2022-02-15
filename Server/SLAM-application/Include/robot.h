#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <list>
#include "animated_line.h"
#include "lines.h"
#include "panel.h"
#include "squares.h"
#include "updatable.h"
#include "path.h"
#include "callbacks.h"
#include "particle.h"
#include "slam_message.h"

namespace NTNU::application::SLAM
{

enum class robot_events
{
	POINTS_CLEARED,
	OBSTACLE
};

class robot :
	public sf::Drawable,
	public sf::Transformable,
	public NTNU::gui::panel::panel,
	public NTNU::gui::base::updatable,
	public NTNU::utility::callbacks<enum robot_events>
{
public:
	robot(const std::string& name, pose_t init_pose, std::array<int16_t, 3> size);
	~robot() {};

	void add_obstacle(const sf::Vector2f& pos);
	std::vector<std::pair<int16_t, int16_t>> get_obstacles() const;

	void setOffset(sf::Vector2f offset);

	void setSize(float size);
	float getSize() const;
	std::string getName() const;
	sf::Color getColor() const;
	void setColor(sf::Color color);

	void set_path(const std::vector<std::pair<float, float>>& coords);
	pose_t pose();

	std::optional<std::pair<float, float>> get_target() const;
	std::optional<std::pair<float, float>> get_next_on_path() const;

	void remove_old_point();
	void clear();
	void clear_path();

	void update(pose_t new_pose, std::vector<message::position> obs, std::vector<bool> is_object);
	void update();
	void update(sf::Time delta) override;
	void draw(sf::RenderTarget & target, sf::RenderStates states) const override;

private:
	std::string name_;
	float robot_size_;
	sf::Color color_;
	NTNU::gui::collections::squares squares_;
	sf::CircleShape robot_marker_;
	std::list<animated_line> animated_lines_;
	NTNU::gui::collections::lines lines_;
	NTNU::gui::base::path path_;
	sf::Texture robot_texture_;
	sf::Sprite robot_sprite_;
	float robot_scaling_;
	pose_t odom_;
	particleFilter particles_;
	pose_t prev_pose_;
	bool scanning_;
	std::vector<message::position> scan_points_;
	std::vector<bool> is_object_list_;
};

}

