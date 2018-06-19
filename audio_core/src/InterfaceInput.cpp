/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#include "debug.h"
#include "InterfaceInput.h"
#include <ros/ros.h>
#include <audio_msg/AudioBuffer.h>
#include <boost/thread.hpp>

appl::InterfaceInput::InterfaceInput(std11::shared_ptr<audio::river::Manager> _manager,
                                     const etk::String& _input,
                                     const etk::String& _publisher,
                                     bool _feedback) :
  m_manager(_manager) {
	ros::NodeHandle nodeHandlePrivate("~");
	m_stream = nodeHandlePrivate.advertise<audio_msg::AudioBuffer>(_publisher,
	                                                               100,
	                                                               boost::bind(&InterfaceInput::onConnect, this, _1),
	                                                               boost::bind(&InterfaceInput::onDisConnect, this, _1));
	//Set stereo output:
	etk::Vector<audio::channel> channelMap;
	channelMap.pushBack(audio::channel_frontLeft);
	channelMap.pushBack(audio::channel_frontRight);
	if (_feedback == false) {
		m_interface = m_manager->createInput(48000,
		                                     channelMap,
		                                     audio::format_int16,
		                                     _input);
	} else {
		m_interface = m_manager->createFeedback(48000,
		                                        channelMap,
		                                        audio::format_int16,
		                                        _input);
	}
	if(m_interface == null) {
		APPL_ERROR("null interface");
		return;
	}
	// set callback mode ...
	m_interface->setInputCallback(std11::bind(&InterfaceInput::onDataReceived,
	                                          this,
	                                          std11::placeholders::_1,
	                                          std11::placeholders::_2,
	                                          std11::placeholders::_3,
	                                          std11::placeholders::_4,
	                                          std11::placeholders::_5,
	                                          std11::placeholders::_6));
	m_interface->start();
}
appl::InterfaceInput::~InterfaceInput() {
	if(m_interface == null) {
		APPL_ERROR("null interface");
		return;
	}
	m_interface->stop();
	m_interface.reset();
}
void appl::InterfaceInput::onConnect(const ros::SingleSubscriberPublisher& _pub) {
	APPL_ERROR("on connect ... " << _pub.getSubscriberName());
}
void appl::InterfaceInput::onDisConnect(const ros::SingleSubscriberPublisher& _pub) {
	APPL_ERROR("on dis-connect ... " << _pub.getSubscriberName());
}
void appl::InterfaceInput::onDataReceived(const void* _data,
                                          const audio::Time& _time,
                                          size_t _nbChunk,
                                          enum audio::format _format,
                                          uint32_t _frequency,
                                          const etk::Vector<audio::channel>& _map) {
	if (_format != audio::format_int16) {
		APPL_ERROR("call wrong type ... (need int16_t)");
	}
	// check if someone suscribe to the output:
	if (m_stream.getNumSubscribers() > 0) {
		audio_msg::AudioBuffer msg;
		// create the Ros timestamp
		msg.header.stamp = ros::Time::now();//_time;
		// set message frequency
		msg.frequency = _frequency;
		// set channel map properties
		msg.channelMap = audio::convertChannel(_map);
		// Set the format of flow
		msg.channelFormat = audio::convertFormat(_format);
		// copy data:
		msg.data.resize(_nbChunk*_map.size()*audio::getFormatBytes(_format));
		// copy dat aon the ROS buffer interface
		memcpy(&msg.data[0], _data, _nbChunk*_map.size()*audio::getFormatBytes(_format));
		// publish message
		m_stream.publish(msg);
	}
}