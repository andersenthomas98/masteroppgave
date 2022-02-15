#include "robots_outbox_handler.h"
#include "log.h"

/*
Robots outbox thread
*/

template <typename T>
bool IsInBounds(const T& value, const T& low, const T& high) {
	return !(value < low) && (value < high);
}

void robots_outbox_thread(NTNU::application::SLAM::robots* robots, NTNU::gui::panel::target_panel* target_panel, boost::fibers::buffered_channel<NTNU::application::SLAM::message>* mqtt_to_publish_ch) {
	//Create vector containing last sent point from which robot id
	std::vector<std::pair<std::pair<int, int>, std::string>> prev;
	
	for (;;)
	{
		auto all_robots = robots->get_all_robot_ids();

		for (const auto& robot : all_robots)
		{

			std::optional<ipair> proposed_point = std::nullopt;

			if (target_panel->get_manual_robot_drive())
			{
				proposed_point = target_panel->get_manual_target();
				auto [nx, ny] = proposed_point.value();

			}

			if (robots->search_grid())
			{
				auto current_position = robots->position(robot);
				auto [c_x, c_y] = current_position.value();

				auto moved = false;

				//Check if robot has moved
				//Fix bug. If robot is i.e. stuck against a wall and consequently does not move,
				//bug causes server to never send new targets.
				if (prev.empty()) {
					prev.push_back(std::pair<std::pair<int, int>, std::string>{current_position.value(), robot});
					moved = true;
				}
				else {
					for (auto& p : prev) {
						if (p.second == robot) {
							if (p.first != std::pair<int, int>(c_x, c_y))
								moved = true;
						}
					}
				}

				if (moved) {
					//If it has, add new random point to path
					auto grid_dim = robots->get_grid_dim();

					auto grid_pos = NTNU::application::SLAM::utility::coord_to_row_col(std::array<int16_t, 2>({ grid_dim[0], grid_dim[1] }), grid_dim[2], c_x, c_y);

					auto [new_row, new_col] = grid_pos.value();

					auto selection1 = NTNU::application::SLAM::utility::get_random(0, 1);
					auto selection2 = NTNU::application::SLAM::utility::get_random(0, 1);

					if (selection1 <= 0)
						if (selection2 <= 0)
							new_row += 40;
						else
							new_row -= 40;
					else {
						if (selection2 <= 0)
							new_col += 40;
						else
							new_col -= 40;
					}


					auto new_target = std::pair<int16_t, int16_t>(new_row, new_col);

					robots->update_path_for_robot(robot, new_target);

					proposed_point = robots->get_next_point(robot);
				}
			}

			auto pending_point = robots->get_next_point(robot);

			// Send new point from path if robot has moved
			// For the case of manual point added
			if (pending_point) {

				auto current_position = robots->position(robot);
				auto [c_x, c_y] = current_position.value();

				if (prev.empty()) {
					prev.push_back(std::pair<std::pair<int, int>, std::string>{current_position.value(), robot});
					proposed_point = pending_point.value();
				}
				else {
					//Check if robot has moved
					for (auto& p : prev) {
						if (p.second == robot) {
							if (p.first == std::pair<int,int>(c_x, c_y)) {
								//If not, do nothing
								break;
							}
							else if (std::sqrt(pow(c_x - pending_point.value().first , 2) + pow(c_y - pending_point.value().second, 2)) > 100 ) {
								//If it has, update position and send new point if distance is far enough
								p.first = std::pair<int, int>(c_x, c_y);
								proposed_point = pending_point.value();
							}
							else {
								//If new point too close to current position, remove point
								robots->remove_old_point(robot);
								break;
							}
						}
					}
				}
			}
			

			if (!proposed_point) {
				continue;
			}

			//Her sendes samme til alle
			auto result = NTNU::networking::protocols::MQTT::utility::parse_topic(robot);
			if (result)
			{
				auto mqtt_topic = result.value();

				NTNU::application::SLAM::message msg(mqtt_topic.version + "/server/" + mqtt_topic.id + "/cmd");

				auto [nx, ny] = proposed_point.value();

				//Set as simulation data
				auto simulated = false;
				if (mqtt_topic.id == std::string("simulated"))
					simulated = true;

				NTNU::application::SLAM::message::position pos{ nx, ny };
				msg.set_payload(pos, simulated);
				
				//std::cout << "Outbox next point, X: " << pos.x << " Y: " << pos.y << std::endl;
				LOG_INFO("Outbox next point, x: {} y: {}", pos.x, pos.y);
				auto result = mqtt_to_publish_ch->push(msg);
				if (result != boost::fibers::channel_op_status::success) {
					//std::cerr << "Robot push msg onto publish queue did not succeed!\n";
					LOG_ERROR("Robot push msg onto publisher queue did not succeed!");
				}
				else
					robots->remove_old_point(robot);

			}
			else
			{
				//std::cout << "Bad topic parse?!" << '\n';
				LOG_WARN("Bad topic parse?");
			}

			boost::this_thread::sleep_for(boost::chrono::milliseconds{ 100 });
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds{ 500 });
	}
}