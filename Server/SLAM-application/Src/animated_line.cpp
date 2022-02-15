#include "animated_line.h"
#include <SFML/Graphics.hpp>
#include <Thor/Animations.hpp>
#include <iostream>

void animate_position(animated_line& aline, float progress)
{
	auto to = aline.getTo();
	auto from = aline.getFrom();

	// Want to have two phases.
	// Use a 0.0f --> 1.0f range for both halves.
	auto progress_1st_half = +0.0f + 2.0f * progress;
	auto progress_2nd_half = -1.0f + 2.0f * progress;

	if (progress < 0.5f)
	{
		aline.line_[1].position = from + (progress_1st_half * (to - from));
	}
	else
	{
		// The resolution on the progress is not as smooth as we would like.
		// Therefore the last float value might be 0.42 instead of 0.4999...
		// This leads to the the first half not drawing a long enough line
		// (based on how coarse the attained resolution is).
		// This then saturates the result from the first half directly.
		static bool once = true;
		if (once)
		{
			once = false;
			aline.line_[1].position = to;
		}

		aline.line_[0].position = from + (progress_2nd_half * (to - from));
	}
}

animated_line::animated_line(sf::Vector2f from, sf::Vector2f to, sf::Color color, sf::Time duration) :
	line_(sf::Lines),
	animator_(),
	color_(color)
{
	from_ = from;
	to_ = to;

	sf::Vertex from_v(from);
	sf::Vertex to_v(to);

	from_v.color = color_;
	to_v.color = color_;

	line_.append(from_v);
	line_.append(to_v);

	animator_.addAnimation(0, &animate_position, duration);
	animator_.playAnimation(0, false);
}

animated_line::~animated_line()
{
}

bool animated_line::update(sf::Time dt)
{
	animator_.update(dt);
	animator_.animate(*this);

	return animator_.isPlayingAnimation();
}

sf::Vector2f animated_line::getTo()
{
	return to_;
}

sf::Vector2f animated_line::getFrom()
{
	return from_;
}

void animated_line::setColor(sf::Color color)
{
	for (int i = 0; i < line_.getVertexCount(); i++)
	{
		line_[i].color = color;

	}
}

void animated_line::draw(sf::RenderTarget & target, sf::RenderStates states) const
{
	target.draw(line_, states);
}