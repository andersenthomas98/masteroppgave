#pragma once

#include <boost/fiber/buffered_channel.hpp>

#include "simulation_panel.h"
#include "robot_simulation.h"

void robot_simulation_thread(NTNU::gui::panel::simulation_panel* simulation_panel, NTNU::application::SLAM::robot_simulation_config* config, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* simulator_ch);