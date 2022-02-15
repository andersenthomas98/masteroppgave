#pragma once

#include <string>
#include <vector>
//#include <MQTTAsync.h>
#include <paho-mqtt/MQTTAsync.h>
#include <map>
#include <set>

#include "callbacks.h"

namespace NTNU::networking::protocols::MQTT {

enum class MQTT_events {
	connect_response,
	disconnect_response,
	subscribe_response,
	publish_response,
	connection_lost,
	message_arrived,
	delivery_complete,
};

class MQTT : public NTNU::utility::callbacks<MQTT_events>
{
public:
	enum Result {
		SUCCESS = 0,
		FAIL
	};

	enum QoS {
		AT_MOST_ONCE = 0,
		AT_LEAST_ONCE = 1,
		EXACTLY_ONCE = 2,
	};

	MQTT(const std::string& address, QoS QoS = AT_MOST_ONCE);
	~MQTT();

	Result connect();
	Result disconnect();

	std::string getClientId() const;

	Result subscribe(const std::string& topic);

	Result publish(const std::string& topic, const std::string& message) const;
	Result publish(const std::string& topic, const std::vector<std::byte>& message) const;

private:
	MQTTAsync client_;
	std::string address_;
	std::string id_;
	QoS qos_;
	unsigned int keepAliveInterval_;

	MQTTAsync_connectOptions connOpts_;
	MQTTAsync_disconnectOptions discOpts_;
	void* context_;

	Result publish(const std::string& topic, void * data, std::size_t len) const;
};

}
