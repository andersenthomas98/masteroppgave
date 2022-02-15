#include "MQTT.h"
#include "callbacks.h"
#include <iostream>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "log.h"
#include <sstream>

namespace NTNU::networking::protocols::MQTT {

static void connection_lost(void* context, char* cause) {
	//std::cout << "Connection lost\n";
	LOG_WARN("Connection lost");
	// Workaround C-legacy: Create a 'this' pointer
	MQTT* this_ = (MQTT *)context;
	this_->call_callback(MQTT_events::connection_lost, std::make_any<bool>(false));
}

static int message_arrived(void* context, char* topicName, int topicLen, MQTTAsync_message* message) {
	std::string topic{ topicName };
	std::string payload{ (char *)message->payload, static_cast<unsigned int>(message->payloadlen) };
	auto cb_data = std::make_pair(topic, payload);

	MQTT* this_ = (MQTT *)context;
	this_->call_callback(MQTT_events::message_arrived, cb_data);

	MQTTAsync_freeMessage(&message);
	MQTTAsync_free(topicName);

	return 1;
}

static void delivery_complete(void* context, MQTTAsync_token token) {
	//std::cout << "Delivery complete\n";
	LOG_INFO("Delivery complete");
	MQTT* this_ = (MQTT *)context;
	this_->call_callback(MQTT_events::delivery_complete, std::make_any<bool>(true));
}

MQTT::MQTT(const std::string & address, QoS QoS) :
	client_(),
	address_(address),
	id_(),
	qos_(QoS),
	keepAliveInterval_(10),
	connOpts_(MQTTAsync_connectOptions_initializer),
	discOpts_(MQTTAsync_disconnectOptions_initializer),
	context_(this)
{
	boost::uuids::random_generator gen;
	boost::uuids::uuid id = gen();
	std::stringstream ss;
	ss << "MQTT-SLAM-" << id;
	id_ = ss.str();

	MQTTAsync_create(&client_, address_.c_str(), id_.c_str(), MQTTCLIENT_PERSISTENCE_NONE, NULL);
	connOpts_.cleansession = 1;
}

MQTT::~MQTT()
{
	MQTTAsync_destroy(&client_);
}

MQTT::Result MQTT::connect()
{
	if (MQTTAsync_setCallbacks(client_, context_, connection_lost, message_arrived, delivery_complete) != MQTTASYNC_SUCCESS)
	{
		return FAIL;
	}
	connOpts_.context = context_;

	connOpts_.onSuccess = [](void* context, MQTTAsync_successData* response) {
		MQTT* this_ = (MQTT *)context;
		this_->call_callback(MQTT_events::connect_response, std::make_any<bool>(true));

	};
	connOpts_.onFailure = [](void* context, MQTTAsync_failureData* response) {
		//std::cout << "Connect failure\n";
		LOG_ERROR("Connection failure");

		MQTT* this_ = (MQTT *)context;
		this_->call_callback(MQTT_events::connect_response, std::make_any<bool>(false));
	};

	return (MQTTAsync_connect(client_, &connOpts_) == MQTTASYNC_SUCCESS) ? SUCCESS : FAIL;
}

MQTT::Result MQTT::disconnect()
{
	discOpts_.context = context_;

	discOpts_.onSuccess = [](void* context, MQTTAsync_successData* response) {
		MQTT* this_ = (MQTT *)context;
		this_->call_callback(MQTT_events::disconnect_response, std::make_any<bool>(true));
	};
	discOpts_.onFailure = [](void* context, MQTTAsync_failureData* response) {
		//std::cout << "Disconnect failure\n";
		LOG_ERROR("Disconnect failure");

		MQTT* this_ = (MQTT *)context;
		this_->call_callback(MQTT_events::disconnect_response, std::make_any<bool>(false));
	};

	return (MQTTAsync_disconnect(client_, &discOpts_) == MQTTASYNC_SUCCESS) ? SUCCESS : FAIL;
}

std::string MQTT::getClientId() const
{
	return id_;
}


MQTT::Result MQTT::subscribe(const std::string & topic)
{
	MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;

	struct sub_context {
		MQTT* instance;
		std::string topic;
	};

	// We have to create a context to give the C-style callback,
	// we can not simply pass ourselves in the lambda capture list.
	static struct sub_context context { this, "" };
	context.topic = topic;
	opts.context = (void*)&context;

	opts.onSuccess = [](void* context, MQTTAsync_successData* response) {
		sub_context* sc = (sub_context *)context;

		sc->instance->call_callback(MQTT_events::subscribe_response, sc->topic);
	};
	opts.onFailure = [](void* context, MQTTAsync_failureData* response) {
		//std::cout << "Subscribe failure\n";
		LOG_ERROR("Subsribe failure");
		sub_context* sc = (sub_context *)context;

		sc->instance->call_callback(MQTT_events::subscribe_response, "");
	};

	return MQTTAsync_subscribe(client_, topic.c_str(), qos_, &opts) == MQTTASYNC_SUCCESS ? SUCCESS : FAIL;
}

MQTT::Result MQTT::publish(const std::string & topic, const std::string & message) const
{
	return publish(topic, (void *)message.c_str(), message.size());
}

MQTT::Result MQTT::publish(const std::string & topic, const std::vector<std::byte>& message) const
{
	return publish(topic, (void *)message.data(), message.size());
}

MQTT::Result MQTT::publish(const std::string& topic, void * data, std::size_t len) const
{
	MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
	MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;

	pubmsg.payload = data;
	pubmsg.payloadlen = len;
	pubmsg.qos = qos_;
	pubmsg.retained = 0;

	opts.context = context_;
	opts.onSuccess = [](void* context, MQTTAsync_successData* response) {
		std::string topic{ response->alt.pub.destinationName };
		std::string payload{ (char *)response->alt.pub.message.payload, static_cast<unsigned int>(response->alt.pub.message.payloadlen) };

		auto cb_data = std::make_pair(topic, payload);

		MQTT* this_ = (MQTT *)context;
		this_->call_callback(MQTT_events::publish_response, cb_data);
	};
	opts.onFailure = [](void* context, MQTTAsync_failureData* response) {
		//std::cout << "Publish failure\n";
		LOG_ERROR("Publish failure");

		MQTT* this_ = (MQTT *)context;
		this_->call_callback(MQTT_events::publish_response, false);
	};

	return MQTTAsync_sendMessage(client_, topic.c_str(), &pubmsg, &opts) == MQTTASYNC_SUCCESS ? SUCCESS : FAIL;
}

}
