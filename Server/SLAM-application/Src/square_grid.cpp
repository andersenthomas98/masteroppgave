#include "square_grid.h"
#include <iostream>

using NTNU::gui::collections::squares;

///-------------- Visual part of grid -------------///

namespace NTNU::gui::elements
{

square_grid::square_grid(int rows, int columns, int separation, sf::Color color) :
	rows_(rows),
	columns_(columns),
	separation_(separation)
{
	squares_.setPrimitiveType(sf::Quads);
	reset_to(rows, columns, separation, color);
}

void square_grid::set_color(sf::Color color)
{
	for (auto i = 0; i < squares_.getVertexCount(); i++)
		squares_[i].color = color;
}

void square_grid::change_color(sf::Color color, int row, int column)
{

	sf::Vertex* va = &squares_[(column + row * columns_)*4];

	for (auto i = 0; i < 4; i++)
		va[i].color = color;


}

void square_grid::set_alpha(int alpha)
{
	//squares_->set_alpha(alpha);
}

void square_grid::reset_to(int rows, int columns, int separation, sf::Color color)
{
	//squares_ = std::make_unique<squares>(separation);
	squares_.resize(rows * columns * 4);
	//squares_.setColor(color);
	separation_ = separation;
	/*
	for (int row = 0; row < rows; row++)
	{
		for (int col = 0; col < columns; col++)
		{
			auto x = col * separation;
			auto y = row * separation;
			squares_->add({ col * separation, row * separation });
		}
	}

	squares_->setSize(static_cast<float>(separation));
	*/

	for (unsigned int i = 0; i < columns; ++i)
		for (unsigned int j = 0; j < rows; ++j)
		{
			// get the current tile number
			//int tileNumber = tiles[i + j * width];

			// find its position in the tileset texture
			//int tu = tileNumber % (m_tileset.getSize().x / tileSize.x);
			//int tv = tileNumber / (m_tileset.getSize().x / tileSize.x);

			// get a pointer to the current tile's quad
			sf::Vertex* quad = &squares_[(i + j * columns) * 4];

			// define its 4 corners
			quad[0].position = sf::Vector2f(i * separation_, j * separation_);
			quad[1].position = sf::Vector2f((i + 1) * separation_, j * separation_);
			quad[2].position = sf::Vector2f((i + 1) * separation_, (j + 1) * separation_);
			quad[3].position = sf::Vector2f(i * separation_, (j + 1) * separation_);

			for (auto i = 0; i < 4; i++) {
				quad[i].color = color;
			}

			// define its 4 texture coordinates
			//quad[0].texCoords = sf::Vector2f(tu * tileSize.x, tv * tileSize.y);
			//quad[1].texCoords = sf::Vector2f((tu + 1) * tileSize.x, tv * tileSize.y);
			//quad[2].texCoords = sf::Vector2f((tu + 1) * tileSize.x, (tv + 1) * tileSize.y);
			//quad[3].texCoords = sf::Vector2f(tu * tileSize.x, (tv + 1) * tileSize.y);
		}
}

void square_grid::draw(sf::RenderTarget & target, sf::RenderStates states) const
{
	states.transform *= getTransform();
	/*
	for (const auto & c : squares_->get())
	{
		target.draw(c, states);
	}
	*/
	target.draw(squares_, states);
}

sf::Vector2f square_grid::size() const
{
	return {
		static_cast<float>(columns_ * separation_),
		static_cast<float>(rows_ * separation_),
	};
}


}
