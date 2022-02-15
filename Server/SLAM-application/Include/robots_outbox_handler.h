#pragma once
// USER
#include "slam_message.h"
#include "robots.h"
#include "robot.h"
#include "MQTT_utility.h"
#include "slam_utility.h"

//PANELS
#include "target_panel.h"

//STDs
#include <algorithm>

// boost
#include <boost/fiber/all.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

// TIME
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;

void robots_outbox_thread(NTNU::application::SLAM::robots* robots, NTNU::gui::panel::target_panel* target_panel, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* mqtt_to_publish_ch);