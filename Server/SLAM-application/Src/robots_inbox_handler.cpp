#include "robots_inbox_handler.h"
#include "log.h"

/*
Robot inbox thread
*/
void robots_inbox_thread(NTNU::application::SLAM::robots* robots, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* slam_ch) {
	for (;;)
	{
		NTNU::application::SLAM::message msg;
		auto res = slam_ch->pop(msg);

		if (res != boost::fibers::channel_op_status::success) {
			//std::cerr << "SLAM ch pop res not success!\n";
			LOG_ERROR("SLAM ch pop res not success");
			continue;
		}

		if (!msg.is_valid()) {
			LOG_ERROR("Got invalid SLAM message!");
			//std::cerr << "Got invalid SLAM message!";
			continue;
		}

		robots->feed_message(msg);

		boost::this_thread::yield();
	}
}