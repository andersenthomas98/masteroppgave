#include "robot.h"
#include <iostream>

#include "imconfig-SFML.h"
#include <imgui.h>
#include "imgui-SFML.h"
#include "log.h"
#include <Thor/Graphics.hpp>
#include <Thor/Math.hpp>
#include <Thor/Animations.hpp>
#include <Thor/Vectors.hpp>
#include "particle.h"
#include "slam_utility.h"


namespace NTNU::application::SLAM
{

robot::robot(const std::string& name, pose_t init_pose, std::array<int16_t, 3> size) :
	name_(name),
	robot_size_(10.f),
	color_(),
	squares_(size[2]), //Separation
	robot_marker_(5.0f, 16),
	animated_lines_(),
	lines_(),
	path_(),
	robot_scaling_(5),
	particles_(name, init_pose, std::array{ 0.001, 0.00001, 0.0001, 0.001 }, std::array{ size[0], size[1], size[2], (int16_t)30 }), //Real map (1) //Init pose, std.dev parameters, {//rows,cols, //separation, numParticles}
	odom_(),
	prev_pose_(init_pose),
	scanning_(true),
	scan_points_()
{

	color_.a = 200;

	robot_marker_.setFillColor(color_);
	robot_marker_.setOrigin({ 5.f , 5.f });
	robot_marker_.setPointCount(3);
	robot_marker_.setRotation(90);
	robot_marker_.setScale(robot_scaling_, robot_scaling_*3);

	squares_.setSize(2.f);
	squares_.setColor(color_);
	squares_.setOrigin({ 2.f, 2.f });
	lines_.set_color(sf::Color::White);

	path_.set_color(color_);

	robot_sprite_.setTexture(robot_texture_);
	auto robot_size = robot_texture_.getSize();
	robot_sprite_.setOrigin(robot_size.x /2 , robot_size.y /2 ); 

	robot_sprite_.setScale(robot_scaling_, robot_scaling_*3);

	set_fun([&]() {
		{
			ImGui::Text(name_.c_str());
		}
		{
			float size = robot_size_;
			if (ImGui::SliderFloat("size", &size, 1.0f, 10.0f))
			{
				setSize(size);
			}
		}
		{
			float rotation = thor::toRadian(getRotation());
			if (ImGui::SliderAngle("Tower Angle", &rotation, 0.f, 359.0f))
			{
				setRotation(thor::toDegree(rotation));
			}
		}
		{
			if (ImGui::SliderFloat("Robot Size", &robot_scaling_, 0.005f, 0.05f))
			{
				robot_sprite_.setScale(robot_scaling_, robot_scaling_);
			}
		}
		{
			if (ImGui::TreeNode("Color"))
			{
				ImVec4 color = ImVec4(color_);
				ImGui::SetColorEditOptions(ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoOptions | ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar);
				if (ImGui::ColorPicker4("Color", (float*)&color))
				{
					setColor(sf::Color(color));
				}
				ImGui::TreePop();
			}
		}
		{
			if (ImGui::Button("Reset"))
			{
				call_callback(robot_events::POINTS_CLEARED, name_);
				clear();
			}
			ImGui::SameLine();
			ImGui::AlignTextToFramePadding();
			ImGui::Text("Points: %d", squares_.get().size());
		}
		{
			if (ImGui::Button("Set auto simulation path"))
			{
				path_.set_auto_sim_path();
			}
		}
	});
}

void robot::update(pose_t odom, std::vector<message::position> obs, std::vector<bool> is_object)
{

	//odom_ = odom;

	/* Real robot */
	odom_ = odom_ + odom;

	LOG_INFO("x: {}, y: {}, theta: {}", odom_.x, odom_.y, odom_.theta);
	
	if (scanning_) {
		scan_points_.insert(scan_points_.end(), obs.begin(), obs.end());
		is_object_list_.insert(is_object_list_.end(), is_object.begin(), is_object.end());
	}

	//Change to update scan, and rather update particle only when scan_border arrives?
}

void robot::update() {
	
	scanning_ = false;

	//Only update if sufficient number of points
	/*if (scan_points_.size() <= 300) {
		scan_points_.clear();
		is_object_list_.clear();
		scanning_ = true;
		return;
	}*/

	particles_.update_particle(odom_, scan_points_, is_object_list_);
	odom_ = { 0,0,0 };
	scan_points_.clear();
	is_object_list_.clear();

	scanning_ = true;

	prev_pose_ = particles_.getPose();

	//std::cout << "New Mean Pose: X: " << prev_pose_.x << " Y: " << prev_pose_.y << " Th: " << prev_pose_.theta << std::endl;
	          
	LOG_FILE("New mean pose: x: {: 10.4g}, y: {: 10.4g}, theta: {: 10.4g}", prev_pose_.x, prev_pose_.y, prev_pose_.theta);

	setPosition(sf::Vector2f(static_cast<float>(prev_pose_.x), static_cast<float>(prev_pose_.y)));
	setRotation(static_cast<float>(prev_pose_.theta));

	auto map = particles_.getBestMap();

	particles_.cleanRaster();

	call_callback(robot_events::OBSTACLE, map);


}

void robot::update(pose_t odom, message::line new_line) {
	odom_ = odom_ + odom;

	std::pair<int, int> start = { new_line.startPoint.x , new_line.startPoint.y };
	std::pair<int, int> end = { new_line.endPoint.x , new_line.endPoint.y };

	auto [length, wall] = NTNU::application::SLAM::utility::get_line_between_pts(start, end);

	std::vector<message::position> obs;
	std::vector<bool> is_object;

	for (auto i = 0; i < length; i++) {
		message::position point_position;
		point_position.x = wall[i].first;
		point_position.y = wall[i].second;
		obs.push_back(point_position);
		is_object.push_back(true);
	}
	
	scan_points_.insert(scan_points_.end(), obs.begin(), obs.end());
	is_object_list_.insert(is_object_list_.end(), is_object.begin(), is_object.end());

	particles_.update_particle(odom_, scan_points_, is_object_list_);
	odom_ = { 0,0,0 };
	scan_points_.clear();
	is_object_list_.clear();

	scanning_ = true;

	prev_pose_ = particles_.getPose();

	//std::cout << "New Mean Pose: X: " << prev_pose_.x << " Y: " << prev_pose_.y << " Th: " << prev_pose_.theta << std::endl;

	LOG_FILE("New mean pose: x: {: 10.4g}, y: {: 10.4g}, theta: {: 10.4g}", prev_pose_.x, prev_pose_.y, prev_pose_.theta);

	setPosition(sf::Vector2f(static_cast<float>(prev_pose_.x), static_cast<float>(prev_pose_.y)));
	setRotation(static_cast<float>(prev_pose_.theta));

	auto map = particles_.getBestMap();

	particles_.cleanRaster();

	call_callback(robot_events::OBSTACLE, map);


}

//This seems to be necessary for some reason
void robot::update(sf::Time delta) {

}

void robot::add_obstacle(const sf::Vector2f& pos)
{
	//Don't believe these visual squares are necessary,
	//but might be nice for debugging
	//If needed, change color from Transparent
	
	squares_.add({ pos.x, pos.y });


	sf::Color col = sf::Color::Transparent;
	//col.a = alpha;
	squares_.setColor(col, squares_.get().size() - 1);
	squares_.setSize(5);

}

pose_t robot::pose() {
	return prev_pose_;
}

std::vector<std::pair<int16_t, int16_t>> robot::get_obstacles() const
{
	//The squares should be retrieved from the particlefilter
	std::vector<std::pair<int16_t, int16_t>> obstacles;
	for (const auto& obs : squares_.get())
	{
		auto pos = obs.getPosition();
		obstacles.emplace_back(pos.x, pos.y);
	}
	return obstacles;
}

void robot::setOffset(sf::Vector2f offset)
{
	Transformable::setPosition(offset);
}

void robot::setSize(float size)
{
	if (size < 1)
	{
		return;
	}

	robot_size_ = size;
	squares_.setSize(size);
}

float robot::getSize() const
{
	return robot_size_;
}

std::string robot::getName() const
{
	return name_;
}

sf::Color robot::getColor() const
{
	return color_;
}


void robot::setColor(sf::Color color)
{
	color_ = color;
	auto opaque_color = color_;
	opaque_color.a = 255;

	robot_marker_.setFillColor(opaque_color);
	squares_.setColor(color_);
	path_.set_color(color_);
	robot_sprite_.setColor(color_);
}

void robot::set_path(const std::vector<std::pair<float, float>>& coords)
{
	path_.clear();
	path_.set(coords);
}

std::optional<std::pair<float, float>> robot::get_target() const
{
	if (path_.size() == 0)
		return std::nullopt;

	return path_.end_point();
}

std::optional<std::pair<float, float>> robot::get_next_on_path() const
{
	if (path_.size() == 0)
		return std::nullopt;

	return path_.next_point();
}

void robot::remove_old_point()
{
	path_.remove_point();
}

void robot::clear_path()
{
	path_.full_clear();
}

void robot::clear()
{
	squares_.clear();
}

void robot::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
	// We want the obstacles to not accept the global passed down
	// transform, but not be affected by own transform...

	//target.draw(squares_, states);
	//target.draw(lines_, states);
	target.draw(path_, states);
	/*
	for (const auto & l : animated_lines_)
	{
		target.draw(l, states);
	}
	*/
	// ... so we apply our own transform here.
	states.transform *= getTransform();
	target.draw(robot_marker_, states);
	target.draw(robot_sprite_, states);
}

}
