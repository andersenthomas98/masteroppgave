#pragma once

#include <SFML/Graphics.hpp>
#include <Thor/Animations.hpp>

class animated_line : public sf::Drawable
{
public:
	animated_line(sf::Vector2f from, sf::Vector2f to, sf::Color color, sf::Time duration);
	~animated_line();

	bool update(sf::Time delta);

	sf::Vector2f getTo();
	sf::Vector2f getFrom();

	void setColor(sf::Color color);

	void draw(sf::RenderTarget &target, sf::RenderStates states) const;

	sf::VertexArray line_;

private:
	sf::Vector2f from_;
	sf::Vector2f to_;
	thor::Animator<animated_line, int> animator_;
	sf::Color color_;
};

