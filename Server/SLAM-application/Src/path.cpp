#include "path.h"
#include "slam_utility.h"
#include <iostream>

namespace NTNU::gui::base
{

path::path(int line_thickness, int square_radius, sf::Color color) :
	lines_(line_thickness),
	squares_(20),
	separation_(20), //separation
	size_(std::array<int16_t,2>{500 , 500}), //rows, cols
	color_(color)
{
	set_color(color);

	squares_.setSize(square_radius);
	auto offset = static_cast<float>(square_radius);
	squares_.setOrigin({ offset, offset });
	

	
}

void path::set(const std::vector<std::pair<float, float>> coords)
{
	auto it = coords.cbegin();
	auto it_ahead = it + 1;

	path_vector_.insert(path_vector_.end(), coords.begin() + 1, coords.end());

	for (it, it_ahead; it_ahead != coords.cend(); it++, it_ahead++)
	{
		auto[x, y] = *it;
		auto[x_ahead, y_ahead] = *it_ahead;

		//std::cout << "Adding point: " << x_ahead << ", " << y_ahead << std::endl;

		lines_.add({ x, y }, { x_ahead, y_ahead });
		squares_.add({ x_ahead, y_ahead });
	}
}

void path::set_color(sf::Color color)
{
	color_ = color;
	lines_.set_color(color);
	squares_.setColor(color);
}

int16_t path::size() const
{
	return static_cast<int16_t>(squares_.get().size());
}

void path::remove_point()
{
	if (!path_vector_.empty())
		path_vector_.erase(path_vector_.begin());
}

std::optional<std::pair<float, float>> path::end_point() const
{
	// The last square on the path should be positioned at the end point.
	//auto last_square = squares_.get().back().getPosition();

	if (path_vector_.empty())
		return std::nullopt;

	auto last_point = path_vector_[path_vector_.size() - 1];

	//auto last_point_grid = NTNU::application::SLAM::utility::row_col_to_coord(size_, separation_, last_point.first, last_point.second);

	return last_point;

}

std::optional<std::pair<float, float>> path::next_point() const
{
	// Since a square is not added at the point of the robot,
	// the "first square is the next square".
	//auto next_square = squares_.get().front().getPosition();

	//Add nullopt!
	if (path_vector_.empty())
		return std::nullopt;

	auto next_point = path_vector_[0];

	//std::cout << "Next point: " << next_point.first << next_point.second << std::endl;

	//auto last_point_grid = NTNU::application::SLAM::utility::row_col_to_coord(size_, separation_, last_point.first, last_point.second);

	return next_point;


	//return { next_square.x, next_square.y };
}

std::vector<std::pair<float, float>> path::get_path() const
{
	return path_vector_;
}

void path::clear()
{
	lines_.clear();
	squares_.clear();
}

void path::full_clear()
{
	//Clearing any existing targets
	lines_.clear();
	squares_.clear();
	if (!path_vector_.empty())
		path_vector_.clear();
}

void path::draw(sf::RenderTarget & target, sf::RenderStates states) const
{
	states.transform *= getTransform();

	target.draw(lines_, states);
	target.draw(squares_, states);
}

void path::set_auto_sim_path()
{
	/* SIMULATION PATH */
	auto x = 400;
	auto y = 0;

	for (x; x < 800; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -800; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x > 0; x = x - 400)
		path_vector_.push_back({ x,y });

	for (x; x < 1600; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -1600; y = y - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 1600; y = y + 400)
		path_vector_.push_back({ x,y });


	for (x; x > 400; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 2400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (y; y > 1600; y = y - 400)
		path_vector_.push_back({ x,y });


	for (x; x > -1600; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > -1600; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < 400; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -2400; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < 2800; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y < -400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x < 3200; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -1600; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < 4000; x = x + 400)
		path_vector_.push_back({ x,y });

	for (x; x > 3200; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y < -400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x > 2800; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x < 4000; x = x + 400)
		path_vector_.push_back({ x,y });


	for (y; y < 1600; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x > 3200; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > 400; y = y - 400)
		path_vector_.push_back({ x,y });


	for (x; x > 2800; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 2400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x < 4000; x = x + 400)
		path_vector_.push_back({ x,y });

	for (x; x > 2800; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 3200; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x < 4000; x = x + 400)
		path_vector_.push_back({ x,y });


	for (y; y < 4000; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x > 3200; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > 3200; y = y - 400)
		path_vector_.push_back({ x,y });


	for (x; x > 2800; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 4000; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x > 1600; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > 2400; y = y - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 3200; y = y + 400)
		path_vector_.push_back({ x,y });



	for (x; x > -800; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > 2400; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x > -2400; x = x - 400)
		path_vector_.push_back({ x,y });



	for (y; y > 1600; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < -1600; x = x + 400)
		path_vector_.push_back({ x,y });

	for (x; x > -2400; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 2400; y = y + 400)
		path_vector_.push_back({ x,y });


	for (x; x < -400; x = x + 400)
		path_vector_.push_back({ x,y });
	//the below replaces the above for long range. Meaning, remove below for original path
	//for (x; x < -1600; x = x + 400)
	//	path_vector_.push_back({ x,y });

	for (y; y < 4400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x > -1600; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > 3200; y = y - 400)
		path_vector_.push_back({ x,y });

	for (y; y < 4400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (x; x > -4000; x = x - 400)
		path_vector_.push_back({ x,y });
	//below replaces above for long range
	//for (x; x > -3200; x = x - 400)
	//	path_vector_.push_back({ x,y });

	for (x; x < -3200; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -800; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x > -4400; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > -1200; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < -3200; x = x + 400)
		path_vector_.push_back({ x,y });

	for (x; x > -4400; x = x - 400)
		path_vector_.push_back({ x,y });

	for (y; y > -4400; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < -3600; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y < -3600; y = y + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -4400; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < -2400; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y < -400; y = y + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -4400; y = y - 400)
		path_vector_.push_back({ x,y });

	for (x; x < 800; x = x + 400)
		path_vector_.push_back({ x,y });

	for (y; y < -1600; y = y + 400)
		path_vector_.push_back({ x,y });

	for (y; y > -4000; y = y - 400)
		path_vector_.push_back({ x,y });


	for (x; x < 4000; x = x + 400)
		path_vector_.push_back({ x,y });


	for (y; y < -2400; y = y + 400)
		path_vector_.push_back({ x,y });
}

}
