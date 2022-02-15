#pragma once

// USER
#include "MQTT.h"
#include "slam_message.h"

// logging
#include "log.h"

//	Panels
#include "MQTT_panel.h"

// C++ standard lib
#include <vector>		// std::vector
#include <utility>		// std::pair, std::any

// boost
#include <boost/fiber/all.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

// FIBERS
using boost::this_fiber::yield;
using boost::fibers::fiber;
using boost::this_fiber::sleep_until;

// TIME
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;

void mqtt_thread(NTNU::gui::panel::mqtt_panel* mqtt_panel, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* slam_ch, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* mqtt_to_publish_ch, boost::fibers::buffered_channel<std::string>* mqtt_to_subscribe_ch, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* simulator_ch);