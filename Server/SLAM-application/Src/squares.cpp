#include "squares.h"
#include <iostream>

//Squares representing measurements
namespace NTNU::gui::collections
{

squares::squares(int separation) :
	squares_(),
	separation_(separation),
	size_(),
	color_()
{
}

void squares::add(std::pair<float, float> pos)
{
	auto square = sf::RectangleShape(sf::Vector2f(separation_, separation_));
	square.setPosition({ pos.first, pos.second });
	square.setFillColor(color_);
	squares_.push_back(square);

	//Dont need to fix this because there is no need for it at all. Might be nice for debugging,
	//so I'll leave it for now
}

//Should be defined by separation?
void squares::setSize(float size)
{
	size_ = sf::Vector2f(size, size);

	for (auto & c : squares_)
	{
		c.setSize(size_);
	}
}


void squares::setColor(sf::Color color)
{
	color_ = color;

	for (auto & c : squares_)
	{
		c.setFillColor(color);
	}
}

void squares::setColor(sf::Color color, int index)
{
	if (index < squares_.size())
		squares_.at(index).setFillColor(color);
}

void squares::set_alpha(int alpha)
{
	for (auto & c : squares_)
	{
		auto color = c.getFillColor();
		color.a = alpha;
		c.setFillColor(color);
	}
}

sf::Color squares::color() const
{
	return color_;
}

const std::vector<sf::RectangleShape>& squares::get() const
{
	return squares_;
}

void squares::clear()
{
	squares_.clear();
}

void squares::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
	states.transform *= getTransform();

	for (const auto& c : squares_)
	{
		target.draw(c, states);
	}
}

}

