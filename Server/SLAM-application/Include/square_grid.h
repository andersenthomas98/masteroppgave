#pragma once

#include "squares.h"
#include <SFML/Graphics.hpp>
#include <memory>

namespace NTNU::gui::elements
{

class square_grid : public sf::Transformable, public sf::Drawable
{
public:
	square_grid(int rows, int columns, int separation, sf::Color color = sf::Color::Black);

	void set_color(sf::Color color);
	void change_color(sf::Color color, int row, int column);
	void set_alpha(int alpha);

	void reset_to(int rows, int columns, int separation, sf::Color color = sf::Color::Black);
	sf::Vector2f size() const;

	void draw(sf::RenderTarget& target, sf::RenderStates states) const override;

protected:
	int rows_;
	int columns_;
	int separation_;

private:
	//std::unique_ptr<gui::collections::squares> squares_;
	sf::VertexArray squares_;
};

}

