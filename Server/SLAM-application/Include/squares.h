#pragma once

#include <utility>
#include <vector>
#include <SFML/Graphics.hpp>

namespace NTNU::gui::collections
{

class squares : public sf::Drawable, public sf::Transformable
{
public:

	squares(int separation);

	void add(std::pair<float, float> pos);

	void setSize(float size);
	//void setNumPoints(int points);
	void setColor(sf::Color color);
	void setColor(sf::Color color, int index);
	void set_alpha(int alpha);

	sf::Color color() const;

	const std::vector<sf::RectangleShape>& get() const;

	void clear();
	
	void draw(sf::RenderTarget& target, sf::RenderStates states) const override;

private:
	std::vector<sf::RectangleShape> squares_;
	//sf::VertexArray squares_;
	int separation_;
	sf::Vector2f size_;
	sf::Color color_;
};

}

