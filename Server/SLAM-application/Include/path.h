#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <array>
#include <optional>
#include "squares.h"
#include "lines.h"

namespace NTNU::gui::base
{

class path : public sf::Drawable, public sf::Transformable
{
public:
	path(int line_thickness = 1, int square_radius = 5, sf::Color = sf::Color::Green);

	void set(const std::vector<std::pair<float, float>> coords);
	void set_color(sf::Color color);
	std::vector<std::pair<float, float>> get_path() const;
	void remove_point();

	int16_t size() const;

	std::optional<std::pair<float, float>> next_point() const;
	std::optional<std::pair<float, float>> end_point() const;

	void clear();
	void full_clear();

	void set_auto_sim_path();

	~path() {};

	void draw(sf::RenderTarget& target, sf::RenderStates states) const override;

private:
	gui::collections::squares squares_;
	gui::collections::lines lines_;
	sf::Color color_;
	std::vector<std::pair<float, float>> path_vector_;
	std::array<int16_t, 2> size_;
	int16_t separation_;
};

}
