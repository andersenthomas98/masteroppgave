#include "window.h"

#include <iostream>

#include "imconfig-SFML.h"
#include <imgui.h>
#include "log.h"
#include "imgui-SFML.h"

namespace NTNU::gui::base
{

window::window() :
	elements_(),
	window_(),
	clk_()
{
	auto mode = sf::VideoMode::getDesktopMode();
	window_.create(mode, window_title_, sf::Style::Resize);

	ImGui::SFML::Init(window_);

	window_.setVerticalSyncEnabled(true);
	move(sf::Vector2f(mode.width / 2.f, mode.height / 2.f));
}

void window::add_panel(const NTNU::gui::panel::panel* panel)
{
	panels_.push_back(panel);
}

void window::add_drawable(const sf::Drawable* element)
{
	elements_.push_back(element);
}

void window::add_updatable(NTNU::gui::base::updatable * thing)
{
	updatables_.push_back(thing);
}

void window::remove_drawable(const sf::Drawable* remove)
{
	for (auto it = elements_.cbegin(); it != elements_.cend(); it++)
	{
		if (*it == remove) {
			elements_.erase(it);
			break;
		}
	}
}

void window::close()
{
	if (window_.isOpen())
		window_.close();
}

bool window::run()
{
	if (window_.isOpen() == false) {
		return false;
	}

	check_events();

	sf::Time delta = clk_.restart();
	ImGui::SFML::Update(window_, delta);
	//	ImGui::ShowTestWindow();

	window_.clear(sf::Color::White);

	try
	{
		for (const auto & thing : updatables_)
		{
			thing->update(delta);
		}
	}
	catch (const std::exception& e)
	{
		//std::cerr << "Updatebles error: " << e.what();
		LOG_ERROR("Updatables error: {}", e.what());
	}

	try
	{
		for (const auto & element : elements_)
		{
			window_.draw(*element, getTransform());
		}
	}
	catch (const std::exception& e)
	{
		//std::cerr << "Draw error: " << e.what();
		LOG_ERROR("Draw error: {}", e.what());
	}

	try
	{
		for (const auto & panel : panels_)
		{
			panel->show();
		}
	}
	catch (const std::exception& e)
	{
		LOG_ERROR("Panels error: {}", e.what());
		//std::cerr << "Panels error: " << e.what();
	}


	ImGui::SFML::Render(window_);
	window_.display();
	return true;
}

void window::check_events()
{
	sf::Event event;
	while (window_.pollEvent(event))
	{
		ImGui::SFML::ProcessEvent(event);

		call_callback(event.type, event);
	}
}

}

