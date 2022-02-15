#pragma once
#include "panel.h"
#include "window.h"
#include "slam_grid.h"
#include "slam_utility.h"
#include "clicks.h"
#include "path.h"
#include "grid_path_solver.h"
#include "robots.h"


//	Panels
#include "robot_simulation.h"
#include "control_panel.h"
#include "target_panel.h"
#include "simulation_panel.h"
#include "MQTT_panel.h"

// C++ standard lib
#include <utility>		// std::pair, std::any
#include <iostream>

// THIRD-PARTY
// imgui
#include "imgui.h"
#include <Thor/Math.hpp>

// boost
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

void gui_thread(NTNU::gui::panel::control_panel* ctrl_panel, NTNU::gui::panel::mqtt_panel* mqtt_panel, NTNU::gui::panel::target_panel* target_panel, NTNU::gui::panel::clicks* clicks, NTNU::gui::panel::simulation_panel* simulation_panel, NTNU::application::SLAM::robots* robots, NTNU::application::SLAM::slam_grid* grid);