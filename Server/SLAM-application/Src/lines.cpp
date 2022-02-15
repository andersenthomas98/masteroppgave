#include "lines.h"
#include <iostream>
#include <Thor/Vectors.hpp>

namespace NTNU::gui::collections
{

lines::lines(int thickness) :
	thickness_(thickness),
	color_(sf::Color::White)
{
}

void lines::add(sf::Vector2f from, sf::Vector2f to)
{
	auto vec = to - from;
	if (vec == sf::Vector2f())
		return;

	auto len = thor::length(to - from);

	auto rec = sf::RectangleShape({ len, static_cast<float>(thickness_) });
	rec.setRotation(thor::polarAngle(vec));
	rec.setPosition(from);
	rec.setFillColor(color_);

	lines_.push_back(rec);
}

void lines::draw(sf::RenderTarget & target, sf::RenderStates states) const
{
	states.transform *= getTransform();

	for (const auto & l : lines_)
	{
		target.draw(l, states);
	}
}

void lines::set_color(sf::Color color)
{
	color_ = color;
	for (auto& l : lines_)
	{
		l.setFillColor(color);
	}
}

void lines::clear()
{
	lines_.clear();
}

}
