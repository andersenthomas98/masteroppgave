#pragma once

#include "obstructable_grid.h"
#include "square_grid.h"
#include "panel.h"
#include "callbacks.h"
#include <utility>
#include <map>
#include <SFML/Graphics.hpp>

namespace NTNU::application::SLAM
{

enum class slam_grid_events
{
	GRID_GEOMETRY_CHANGED,
	NODE_OBSTRUCTED,
};

class slam_grid :
	public NTNU::gui::panel::panel,
	public NTNU::graph::grid::obstructable_grid,
	public NTNU::gui::elements::square_grid,
	public NTNU::utility::callbacks<slam_grid_events>
{
public:
	slam_grid(int16_t rows, int16_t cols, int16_t separation, sf::Color color);

	int16_t separation() const;
	int16_t rows() const;
	int16_t cols() const;

	void set_alpha(int16_t alpha);

	void reset_to(int16_t rows, int16_t cols, int16_t separation);

	void obstruct(int16_t row, int16_t col) override;
	void unobstruct(int16_t row, int16_t col) override;
	void semi_obstruct(int16_t row, int16_t col) override;
	void single_reset(int16_t row, int16_t col);

	virtual ~slam_grid() {};

private:
	sf::Color color_;
	int separation_;
};

}

