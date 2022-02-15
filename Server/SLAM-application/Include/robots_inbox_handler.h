#pragma once
// USER
#include "slam_message.h"
#include "robots.h"

// boost
#include <boost/fiber/all.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

// TIME
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;

void robots_inbox_thread(NTNU::application::SLAM::robots* robots, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* slam_ch);