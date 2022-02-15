#include "slam_grid.h"
#include "imgui.h"
#include <iostream>

using NTNU::graph::grid::obstructable_grid;
using NTNU::gui::elements::square_grid;

///------------- Operations to modify grid ----------------//

namespace NTNU::application::SLAM
{

slam_grid::slam_grid(int16_t rows, int16_t cols, int16_t separation, sf::Color color) :
	obstructable_grid(rows, cols),
	color_(color),
	square_grid(rows, cols, separation, color_),
	separation_(separation)
{
	reset_to(rows, cols, separation);

	set_fun([&]() {
		bool need_reset = false;

		need_reset |= ImGui::SliderInt("Rows", &rows_, 4, 100, "%d");
		need_reset |= ImGui::SliderInt("Columns", &columns_, 4, 100, "%d");
		need_reset |= ImGui::SliderInt("Separation", &separation_, 1, 100, "%d");

		//This it not a sensible slider for this application
		static int alpha = ceil(255/2);
		if (ImGui::SliderInt("Grid Alpha", &alpha, 0, 255, "%d"))
		{
			set_alpha(alpha);
		}

		if (need_reset)
			reset_to(rows_, columns_, separation_);
	});
}

int16_t slam_grid::separation() const
{
	return separation_;
}

int16_t slam_grid::rows() const
{
	return rows_;
}

int16_t slam_grid::cols() const
{
	return columns_;
}

void slam_grid::set_alpha(int16_t alpha)
{
	if (alpha <= 0 || alpha >= 255)
		return;

	color_.a = alpha;

	square_grid::set_alpha(alpha);
}

void slam_grid::reset_to(int16_t rows, int16_t cols, int16_t separation)
{
	rows_ = rows;
	columns_ = cols;
	separation_ = separation;

	square_grid::reset_to(rows, cols, separation);
	auto size = square_grid::size();

	square_grid::setOrigin({ size.x / 2.0f, size.y / 2.0f });
	square_grid::setPosition({ 0.f, 0.f });

	square_grid::set_color(color_);

	obstructable_grid::reset_to(rows, cols);

	call_callback(slam_grid_events::GRID_GEOMETRY_CHANGED, std::any());
}


//Alpha must be set by probability of occupation
void slam_grid::obstruct(int16_t row, int16_t col)
{
	if (!obstructable_grid::is_obstructed(row, col))
	{
		auto obstruct_color = sf::Color::Black;
		obstruct_color.a = 255; //color_.a;

		square_grid::change_color(obstruct_color, row, col);
		obstructable_grid::obstruct(row, col);
		
		//Does below have any effect?
		//call_callback(slam_grid_events::NODE_OBSTRUCTED, (row, col));
	}
}

void slam_grid::unobstruct(int16_t row, int16_t col)
{
	if (!obstructable_grid::is_unobstructed(row, col)) {
		auto unobstruct_color = sf::Color::Black;
		unobstruct_color.a = 1; //color_.a;

		square_grid::change_color(unobstruct_color, row, col);
		//square_grid::set_alpha(0);
		obstructable_grid::unobstruct(row, col);
	}
}

void slam_grid::semi_obstruct(int16_t row, int16_t col)
{
	if (!obstructable_grid::is_semi_obstructed(row, col))
	{
		//auto obstruct_color = sf::Color::Black;
		//obstruct_color.a = 1; //color_.a;

		//square_grid::change_color(obstruct_color, row, col);
		obstructable_grid::semi_obstruct(row, col);

		//Does below have any effect?
		//call_callback(slam_grid_events::NODE_OBSTRUCTED, (row, col));
	}
}

void slam_grid::single_reset(int16_t row, int16_t col)
{
	auto reset_color = sf::Color::Black;
	reset_color.a = ceil(255/2);

	square_grid::change_color(reset_color, row, col);
	obstructable_grid::single_reset(row, col);

}

}
