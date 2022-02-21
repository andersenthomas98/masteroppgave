#include "mqtt_handler.h"

//const std::string MQTT_ADDRESS{ "tcp://188.166.100.22:1883" };

const std::string MQTT_ADDRESS{ "tcp://10.53.51.12:1883" };


/*
MQTT thread
*/
void mqtt_thread(NTNU::gui::panel::mqtt_panel* mqtt_panel,
				boost::fibers::buffered_channel<NTNU::application::SLAM::message>* slam_ch, 
				boost::fibers::buffered_channel<NTNU::application::SLAM::message>* mqtt_to_publish_ch, 
				boost::fibers::buffered_channel<std::string>* mqtt_to_subscribe_ch, 
				boost::fibers::buffered_channel<NTNU::application::SLAM::message>* simulator_ch) 
{
	NTNU::networking::protocols::MQTT::MQTT mqttClient(MQTT_ADDRESS);
	mqtt_panel->set_client_id(mqttClient.getClientId());

	using NTNU::networking::protocols::MQTT::MQTT_events;
	// Shorthand for the format mqtt_panel uses for messages.
	using mqtt_msg = std::pair<std::string, std::string>;

	// Can yield/wait on a response.
	// The first bool indicates if a response has been made (yield while this is false).
	// The second bool is the result of the response (i.e. success/fail).
	std::pair<bool, bool> response{ false, false };

	///////////////////////////////////////////////////
	///// CALLBACKS
	///////////////////////////////////////////////////

	mqttClient.enable_callback(MQTT_events::connect_response, [&](std::any any) {
		try
		{
			auto result = std::any_cast<bool>(any);
			
			if (result) { // successfull connection
				LOG_INFO("MQTT connection respone successfull");
			} 
			else {
				LOG_ERROR("MQTT connection response failed");
			}
			//std::cout << "Connect response result: " << std::boolalpha << result << '\n';

			response = std::make_pair(true, result);
		}
		catch (const std::bad_any_cast & e) { 
			//std::cout << e.what(); 
			LOG_ERROR("Enable callback connect response failed: {}", e.what());
		}
		});

	mqttClient.enable_callback(MQTT_events::message_arrived, [&](std::any any) {
		try
		{
			auto [topic, msg] = std::any_cast<mqtt_msg>(any);
			mqtt_panel->add_msg_in(topic, msg);
			NTNU::application::SLAM::message slam_msg(topic);
			slam_msg.set_payload(msg);

			//std::cout << "Subscribe: " << topic << " | " << msg << '\n';	
				
				// Just to translate and display mqtt message, can be removed //
			/*
				std::vector<std::byte> raw;

				for (const auto& ch : msg) {
					auto byte = std::byte(ch);
					raw.push_back(byte);
				}
				auto iterator = raw.begin();
				std::cout << "pX: " << NTNU::application::SLAM::utility::from_byte_ptr(&(*iterator));
				iterator += 2;
				std::cout << " pY: " << NTNU::application::SLAM::utility::from_byte_ptr(&(*iterator));
				iterator += 2;

				for (iterator; iterator < raw.end();)
				{
					std::cout << " oX: " << NTNU::application::SLAM::utility::from_byte_ptr(&(*iterator));
					iterator += 2;
					std::cout << " oY: " << NTNU::application::SLAM::utility::from_byte_ptr(&(*iterator));
					iterator += 2;
					std::cout << std::endl;
				}
				*/
				// Just to translate and display mqtt message, can be removed //
				

			//std::cout << "Received message type: " << (int)slam_msg.type() << std::endl;

			if (!slam_msg.is_valid()) {
				LOG_ERROR("Received invalid SLAM message!");
				//std::cerr << "Got invalid SLAM message!";
			}

			//Send message to slam_ch if normal message, send to simulator_ch if simulated message
			if (slam_msg.type() == NTNU::application::SLAM::message::msg_type_in::SIM_INIT || slam_msg.type() == NTNU::application::SLAM::message::msg_type_in::SIM_TARGET) {
				if (simulator_ch->try_push(slam_msg) != boost::fibers::channel_op_status::success) {
					LOG_ERROR("Could not put into simulator channel!");
					//std::cerr << "Could not put onto simulator ch!\n";
				}
			}
			else {
				if (slam_ch->try_push(slam_msg) != boost::fibers::channel_op_status::success) {
					//std::cerr << "Could not put onto slam ch!\n";
					LOG_ERROR("Could not put into slam channel!");
				}
			}
		}
		catch (const std::bad_any_cast& e) {
			//std::cout << e.what(); 
			LOG_ERROR("Enable callback message arived failed: {}", e.what());
		}
	});

	mqttClient.enable_callback(MQTT_events::subscribe_response, [&](std::any any) {
		try
		{
			auto result = std::any_cast<std::string>(any);
			//std::cout << "Subscribe response: " << result << '\n';
			LOG_INFO("Subsribe response {}", result);
			if (result != "") {
				mqtt_panel->add_sub(result);
			}
		}
		catch (const std::bad_any_cast& e) {
			LOG_ERROR("Enable callback message arived failed: {}", e.what());
			//std::cout << e.what(); 
		}
	});

	mqttClient.enable_callback(MQTT_events::publish_response, [&](std::any any) {
		try
		{
			auto [topic, msg] = std::any_cast<mqtt_msg>(any);
			//std::cout << "Publish response: " << topic << " | " << msg << '\n';
			mqtt_panel->add_msg_out(topic, msg);
		}
		catch (const std::bad_any_cast & e) { 
			//std::cout << e.what(); 
			LOG_ERROR("{}", e.what());
		}
		});

	if (mqttClient.connect() != NTNU::networking::protocols::MQTT::MQTT::SUCCESS)
	{
		//std::cout << "Connect returned fail!" << std::endl;
		LOG_ERROR("Connect return fail!");
	}

	//Yield thread while waiting for response
	while (!response.first)
		boost::this_thread::yield();

	if (!response.second) {
		LOG_ERROR("Could not connect!");
	}
		//std::cerr << "Could not connect!!\n";

	for (;;)
	{
		// Mqtt: Subscribe to all subtopic of NTNU
		if (static bool once = true; once)
		{
			once = false;

			if (mqttClient.subscribe("v2/robot/#") != NTNU::networking::protocols::MQTT::MQTT::SUCCESS)
			{
				LOG_ERROR("Subsribe to robot error!");
				//std::cerr << "Subscribe to robot error!" << std::endl;
			}

		}

		// Subscribe to outgoing messages to simulator
		if (static bool once = true; once)
		{
			once = false;

			if (mqttClient.subscribe("v2/server/simulated/cmd") != NTNU::networking::protocols::MQTT::MQTT::SUCCESS)
			{
				LOG_ERROR("Subscribe to simulator error!");
				//std::cerr << "Subscribe to simulator error!" << std::endl;
			}

		}

		using NTNU::application::SLAM::message;
		using boost::fibers::channel_op_status;

		// Mqtt: Subscribe to channels if requested to do so
		if (std::string topic; mqtt_to_subscribe_ch->try_pop(topic) ==
			channel_op_status::success) {
			//std::cout << "Got a request to subscribe to: " << topic << '\n';
			LOG_INFO("Got a request to subscribe to: {}", topic);
			if (mqttClient.subscribe(topic) != NTNU::networking::protocols::MQTT::MQTT::SUCCESS)
			{
				LOG_ERROR("Subsribe error from request!");
				//std::cerr << "Subscribe error from request!" << std::endl;
			}
		}

		// Mqtt: Publish messages if requested to do so. Consider validation check?
		if (message msg; mqtt_to_publish_ch->try_pop(msg) ==
			channel_op_status::success) {
			LOG_INFO("Got a request to publish to: {}", msg.sender());
			//std::cout << "Got a request to publish to: " << msg.sender() << '\n';

			if (mqttClient.publish(msg.sender(), msg.serialize()) != NTNU::networking::protocols::MQTT::MQTT::SUCCESS) {
				//std::cout << "Bad publish from request\n";
				LOG_ERROR("Bad publish from request");
			}
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds{ 1000 });
	}
}