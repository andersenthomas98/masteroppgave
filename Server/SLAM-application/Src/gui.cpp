#include "gui.h"
#include "log.h"



void gui_thread(NTNU::gui::panel::control_panel* ctrl_panel, NTNU::gui::panel::mqtt_panel* mqtt_panel, NTNU::gui::panel::target_panel* target_panel, NTNU::gui::panel::clicks* clicks, NTNU::gui::panel::simulation_panel* simulation_panel, NTNU::application::SLAM::robots* robots, NTNU::application::SLAM::slam_grid* grid)
{
	////////////////////////////////////////////////////////////////////////////////
	// GUI: Create window, add children
	////////////////////////////////////////////////////////////////////////////////

	NTNU::gui::base::window win;
	// Flip y-axis by default.
	// This positive y upwards when looking at the screen.
	// This is in line with the robots---the default (y upwards negative) is not.
	// x-axis positive right is still default and correct.
	auto scale = win.getScale();
	scale.y *= -0.1;
	scale.x *= 0.1;
	win.setScale(scale);

	// Drawables
	win.add_drawable(grid);
	win.add_drawable(robots);

	// Updatable
	win.add_updatable(robots);

	NTNU::gui::panel::panel main_panel;
	bool yaxis_flip = true;
	main_panel.set_fun([&win, &yaxis_flip]() {
		if (ImGui::Checkbox("Flip Y-axis", &yaxis_flip)) {
			auto scale = win.getScale();
			auto y = abs(scale.y);
			scale.y = yaxis_flip ? -y : y;
			win.setScale(scale);
		}

		float rotation = thor::toRadian(win.getRotation());
		if (ImGui::SliderAngle("Angle", &rotation, 0.f, 359.0f))
		{
			win.setRotation(thor::toDegree(rotation));
		}
		});

	ctrl_panel->embed_panel(&main_panel, "Main");
	ctrl_panel->embed_panel(mqtt_panel, "MQTT");
	ctrl_panel->embed_panel(clicks, "Clicks");
	ctrl_panel->embed_panel(robots, "Robots");
	ctrl_panel->embed_panel(target_panel, "Manual Drive");
	//ctrl_panel.embed_panel(&grid, "Grid");  Not very useful? Maybe some functions
	ctrl_panel->embed_panel(simulation_panel, "Robot Simulation");


	win.add_panel(ctrl_panel);

	////////////////////////////////////////////////////////////////////////////////
	// Callbacks
	////////////////////////////////////////////////////////////////////////////////

	//Close window
	ctrl_panel->enable_callback(NTNU::gui::panel::control_panel_event::QUIT, [&win](auto a) {
		win.close();
		});

	//Mouse button press options
	win.enable_callback(sf::Event::MouseButtonPressed, [&](sf::Event e) {
		sf::Vector2f pos{ static_cast<float>(e.mouseButton.x), static_cast<float>(e.mouseButton.y) };
		pos = win.getInverseTransform().transformPoint(pos);

		//std::cout << "Mouse button press: {" << pos.x << ", " << pos.y << "}" << std::endl;

		auto grid_idx = NTNU::application::SLAM::utility::coord_to_row_col({ grid->rows() , grid->cols() }, grid->separation(), pos.x, pos.y);
		if (grid_idx)
		{
			using NTNU::gui::panel::clicks_choices;
			const auto [row, col] = grid_idx.value();

			if (e.mouseButton.button == sf::Mouse::Left && clicks->left_is(clicks_choices::obstruct) || e.mouseButton.button == sf::Mouse::Right && clicks->right_is(clicks_choices::obstruct))
			{
				grid->obstruct(row, col);
				//std::cout << "Obstacle: " << row << " , " << col << "\n";
				LOG_INFO("Obstacle: row {}, column {}", row, col);
			}

			if (e.mouseButton.button == sf::Mouse::Left && clicks->left_is(clicks_choices::set_target)
				|| e.mouseButton.button == sf::Mouse::Right && clicks->right_is(clicks_choices::set_target))
			{
				//Sets same target for all robots - should set target for individual robots, for obvious reasons
				//Needs a way to separate
				auto robot_ids = robots->get_all_robot_ids();
				for (const auto& id : robot_ids)
				{
					std::pair<int16_t, int16_t> target = { row, col };
					robots->update_path_for_robot(id, target);
				}
			}
		}
		});

	//Move mouse
	win.enable_callback(sf::Event::MouseMoved, [&grid, &win](sf::Event e) {
		sf::Vector2f pos{ static_cast<float>(e.mouseMove.x), static_cast<float>(e.mouseMove.y) };
		static sf::Vector2f previous{ 0, 0 };
		sf::Vector2f delta = pos - previous;
		previous = pos;

		if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Middle))
		{
			win.move(delta);
		}
		});

	//Scale when scrolling
	win.enable_callback(sf::Event::MouseWheelScrolled, [&win](sf::Event e) {
		constexpr auto scroll_scaling = 0.1;
		auto scroll_amount = e.mouseWheelScroll.delta * scroll_scaling;

		auto scale = win.getScale();
		auto y_sign = std::copysignf(1.0, scale.y);
		auto x_sign = std::copysignf(1.0, scale.x);

		scale.x = abs(scale.x);
		scale.y = abs(scale.y);

		scale.x += scroll_amount*0.1;
		scale.y += scroll_amount*0.1;

		//std::cout << "scale: " << scale.x << "," << scale.y << "\n";

		if (scale.x > 0 && scale.y > 0)
		{
			scale.y *= y_sign;
			scale.x *= x_sign;
			win.setScale(scale);
		}
		});

	//Update GUI
	for (;;)
	{
		if (!win.run()) {
			return;
		}

	}
}