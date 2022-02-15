#pragma once

#include <SFML/System/Time.hpp>

namespace NTNU::gui::base
{

class updatable
{
public:
	virtual void update(sf::Time delta) = 0;

	virtual ~updatable() = 0 {};
};

}
