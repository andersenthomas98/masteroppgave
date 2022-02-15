// USER
#include "panel.h"
#include "lines.h"
#include "window.h"
#include "MQTT.h"
#include "MQTT_utility.h"
#include "slam_grid.h"
#include "slam_message.h"
#include "slam_utility.h"
#include "clicks.h"
#include "path.h"
#include "grid_path_solver.h"
#include "grid_util.h"
#include "robots.h"
#include "robot_simulation.h"
#include "gui.h"
#include "mqtt_handler.h"
#include "robots_inbox_handler.h"
#include "robots_outbox_handler.h"
#include "simulation_handler.h"

//	Panels
#include "control_panel.h"
#include "target_panel.h"
#include "simulation_panel.h"
#include "MQTT_panel.h"
#include "robot_connection_panel.h"
#include "robot_connection_panel.h"

// log
#include "log.h"

// C++ standard lib
#include <vector>		// std::vector
#include <algorithm>	// std::find
#include <utility>		// std::pair, std::any

// THIRD-PARTY
// imgui
#include "imgui.h"
#include <Thor/Math.hpp>

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

// CONFIG
//const std::string MQTT_ADDRESS{"tcp://localhost:1883"};
//	Config when the BoarderRouter is connected to the internet
//const std::string MQTT_ADDRESS{ "tcp://188.166.100.22:1883" };

//const std::string MQTT_ADDRESS{ " tcp://198.41.30.241:1883" };

const std::string MQTT_ADDRESS{ "tcp://10.53.50.43:1883" };


int main()
{
	////////////////////////////////////////////////////////////////////////////////
	// GUI: Setup elements
	////////////////////////////////////////////////////////////////////////////////

	// Drawables
	/* Rows and cols are some places hardcoded and must be manually changed
	 if changed here. Search '//rows' to locate easily */
	int16_t rows = 500;
	int16_t cols = 500;
	int16_t separation = 20;
	auto alpha = ceil(255 / 2);

	auto grid_color = sf::Color::Black;
	grid_color.a = alpha;

	NTNU::application::SLAM::slam_grid grid(rows, cols, separation, grid_color);
	NTNU::application::SLAM::robots robots(&grid);

	// Simulator
	NTNU::application::SLAM::robot_simulation_config config = { MQTT_ADDRESS, "v2/robot/simulated/adv", 5 , std::array<int16_t,3>{rows,cols,separation} };

	// Panels
	NTNU::gui::panel::control_panel ctrl_panel;
	NTNU::gui::panel::mqtt_panel mqtt_panel;
	NTNU::gui::panel::clicks clicks;
	NTNU::gui::panel::target_panel target_panel;
	NTNU::gui::panel::simulation_panel simulation_panel;

	
	////////////////////////////////////////////////////////////////////////////////
	// Setup channels for communication between callbacks and fibers
	////////////////////////////////////////////////////////////////////////////////
	using string_channel = boost::fibers::buffered_channel<std::string>;
	using slam_channel = boost::fibers::buffered_channel<NTNU::application::SLAM::message>;

	slam_channel slam_ch{ 256*2 };
	slam_channel simulator_ch{ 32 };
	slam_channel mqtt_to_publish_ch{ 32 };
	string_channel mqtt_to_subscribe_ch{ 32 };

	//Start timer
	auto t1 = boost::chrono::high_resolution_clock::now();
	
	NTNU::utility::log::init();


	// Create function objects for lambda functions used in several types of callbacks
	auto reapply_obstruction_points = [&](std::any) {
		auto obstacles = robots.get_all_obstacles();

		for (const auto&[x, y] : obstacles)
		{
			//std::cout << "reapply_obstruction_points rows,cols: " << rows << cols << std::endl;
			LOG_INFO("Reapply_obstruction points rows, cols: {}, {}", rows, cols);
			auto convert_to_grid = NTNU::application::SLAM::utility::coord_to_row_col({rows, cols}, separation, x, y);
			if (convert_to_grid)
			{
				auto[row, col] = convert_to_grid.value();
				grid.obstruct(row, col);
			}
		}
	};

	// Create "inline" lambdas for functions used in only one callback
	
	//Maybe use this?
	robots.enable_callback(NTNU::application::SLAM::robots_events::ROBOT_CLEARED_POINTS, [&](std::any context) {
		try
		{
			auto obstacles = std::any_cast<std::vector<std::pair<int, int>>>(context);
			for (const auto&[x, y] : obstacles)
			{
				//std::cout << "robot_cleared_points rows,cols: " << rows << cols << std::endl;
				LOG_INFO("Robot_cleared_points rows, cols: {},{}", rows, cols);
				auto result = NTNU::application::SLAM::utility::coord_to_row_col({ rows, cols }, separation, x, y);
				if (result)
				{
					auto[row, col] = result.value();
					grid.unobstruct(row, col);
				}
			}
		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
	});
	
	robots.enable_callback(NTNU::application::SLAM::robots_events::ROBOT_IDLE, [&](std::any context) {
		try {
			auto [id, x, y] = std::any_cast<std::tuple<std::string, int16_t, int16_t>>(context);
			//std::cout << "Robot [" << id << "] idle at {" << x << ", " << y << "}\n";

		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
		});
		
	robots.enable_callback(NTNU::application::SLAM::robots_events::ROBOT_MOVED, [&](std::any context) {
		try
		{
			auto[id, x, y] = std::any_cast<std::tuple<std::string, int16_t, int16_t>>(context);
			//std::cout << "Robot [" << id << "] moved to {" << x << ", " << y << "}\n";
		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
	});

	robots.enable_callback(NTNU::application::SLAM::robots_events::ROBOT_NEW, [&](std::any context) {
		try
		{
			//auto[id, init_pose]= std::any_cast<std::tuple<std::string, NTNU::application::SLAM::pose_t>>(context);
			auto id = std::any_cast<std::string>(context);

			NTNU::gui::panel::robot_connection_panel robot_connection_panel;

			ctrl_panel.embed_panel(&robot_connection_panel, "New robot connection....");

			//Yield thread while waiting for init pose
			while (!robot_connection_panel.ready())
				boost::this_thread::yield;

			auto init = robot_connection_panel.get_init_pose();

			ctrl_panel.remove_panel();

			NTNU::application::SLAM::pose_t init_pose = { init[0], init[1], init[2] };

			//std::cout << "Init pose selected: " << init[0] << ", " << init[1] << ", " << init[2] << std::endl;

			LOG_INFO("Init pose selected: {}, {}, {}", init[0], init[1], init[2]);

			robots.create_robot(id, init_pose, { rows, cols, separation });

			//Send init pose to robot
			auto result = NTNU::networking::protocols::MQTT::utility::parse_topic(id);
			if (result)
			{
				auto mqtt_topic = result.value();
				auto sim = false;

				NTNU::application::SLAM::message msg(mqtt_topic.version + "/server/" + mqtt_topic.id + "/init");
				
				if (mqtt_topic.id == std::string("simulated"))
					sim = true;

				NTNU::application::SLAM::message::pose pose{ init_pose.x, init_pose.y, init_pose.theta };
				msg.set_payload(pose, sim);
				//std::cout << "Outbox init point, X: " << pose.x << " Y: " << pose.y << " Orient: " << std::endl; // << pose.theta << std::endl;
				auto result = mqtt_to_publish_ch.push(msg);
				if (result != boost::fibers::channel_op_status::success) {
					//std::cerr << "Robot push msg onto publish queue did not succeed!\n";
					LOG_ERROR("Robot push msg onto publish queue did not succeed!");
				}
			}
			else
			{
				//std::cout << "Bad topic parse?!" << '\n';
				LOG_WARN("Bad topic parse?");
			}
		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
		
	});

	robots.enable_callback(NTNU::application::SLAM::robots_events::ROBOT_FOUND_OBSTACLE, [&](std::any context) {
		auto t2 = boost::chrono::high_resolution_clock::now();
		auto duration = boost::chrono::duration_cast<boost::chrono::minutes>(t2 - t1).count();
		//std::cout << "Time of update: " << duration << std::endl;
		LOG_INFO("Time of update: {}", duration);

		try
		{
			auto map = std::any_cast<obstructable_grid>(context);

			//Update map 
			for (int16_t i = 0; i < rows; i++) {
				for (int16_t j = 0; j < cols; j++) {
					grid.single_reset(i, j);
					if (map.is_obstructed(i, j))
						grid.obstruct(i, j);
					else if (map.is_unobstructed(i, j))
						grid.unobstruct(i, j);
					else if (map.is_semi_obstructed(i, j))
						grid.semi_obstruct(i, j);
				}
			}
		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
	});

	//Consider adding a check to see if the topic is valid. Maybe in mqtt_to_publish_ch?
	mqtt_panel.enable_callback(NTNU::gui::panel::mqtt_panel_events::PUBLISH_REQUEST, [&](std::any context) {
		try
		{
			//std::cout << "Publish req\n";
			LOG_INFO("Publish req");
			auto msg = std::any_cast<NTNU::application::SLAM::message>(context);
			if (mqtt_to_publish_ch.try_push(msg) !=
				boost::fibers::channel_op_status::success)
				//std::cerr << "Could not put onto pub ch!\n";
				LOG_ERROR("Could not put onto pub ch!");
		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
	});

	//Same as publish - consider adding a validation check for topic
	mqtt_panel.enable_callback(NTNU::gui::panel::mqtt_panel_events::SUBSCRIBE_REQUEST, [&](std::any context) {
		try
		{
			auto topic = std::any_cast<std::string>(context);
			//std::cout << "Sub req\n";
			LOG_INFO("Sub request");
			if (mqtt_to_subscribe_ch.try_push(topic) !=
				boost::fibers::channel_op_status::success)
				//std::cerr << "Could not put onto sub ch!\n";
				LOG_ERROR("Could not put onto pub ch!");
		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
	});

	grid.enable_callback(NTNU::application::SLAM::slam_grid_events::GRID_GEOMETRY_CHANGED, [&](std::any a) {
		reapply_obstruction_points(a);
	});

	//Nothing?
	grid.enable_callback(NTNU::application::SLAM::slam_grid_events::NODE_OBSTRUCTED, [&](std::any context) {

	});

	////Start threads

	boost::thread_group threads;
	
	threads.add_thread(new boost::thread(gui_thread, &ctrl_panel, &mqtt_panel, &target_panel, &clicks, &simulation_panel, &robots, &grid ));
	threads.add_thread(new boost::thread(mqtt_thread, &mqtt_panel, &slam_ch, &mqtt_to_publish_ch, &mqtt_to_subscribe_ch, &simulator_ch));
	threads.add_thread(new boost::thread(robots_inbox_thread, &robots, &slam_ch));
	threads.add_thread(new boost::thread(robots_outbox_thread, &robots, &target_panel , &mqtt_to_publish_ch));
	threads.add_thread(new boost::thread(robot_simulation_thread, &simulation_panel, &config, &simulator_ch));

	//Wait for all threads to finish
	threads.join_all();

	return 0;
}