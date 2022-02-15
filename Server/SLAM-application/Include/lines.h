#pragma once

#include <utility>
#include <vector>
#include <SFML/Graphics.hpp>

namespace NTNU::gui::collections
{

class lines : public sf::Drawable, public sf::Transformable
{
public:
	lines(int thickness = 1);

	void add(sf::Vector2f from, sf::Vector2f to);
	void draw(sf::RenderTarget& target, sf::RenderStates states) const override;

	void set_color(sf::Color color);

	void clear();

private:
	std::vector<sf::RectangleShape> lines_;
	int thickness_;
	sf::Color color_;
};

}

