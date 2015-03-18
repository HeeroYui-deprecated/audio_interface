

#include <ros/ros.h>
#include "audio_msg/AudioBuffer.h"
#include "audio_core/create.h"
#include "audio_core/remove.h"
#include "audio_core/write.h"
#include "audio_core/getBufferTime.h"
#include <river/river.h>
#include <river/Interface.h>
#include <river/Manager.h>
#include <boost/thread.hpp>
#include <sstream>
#include "debug.h"
#include <etk/stdTools.h>
#include <etk/tool.h>

static std11::mutex mutex;

class InterfaceInput {
	public:
		std11::shared_ptr<river::Manager> m_manager;
		std11::shared_ptr<river::Interface> m_interface;
		ros::Publisher m_stream;
		std11::mutex mutex;
	public:
		InterfaceInput(std11::shared_ptr<river::Manager> _manager, const std::string& _input="microphone", const std::string& _publisher="microphone", bool _feedback=false) :
		  m_manager(_manager) {
			ros::NodeHandle nodeHandlePrivate("~");
			m_stream = nodeHandlePrivate.advertise<audio_msg::AudioBuffer>(_publisher,
			                                                               100,
			                                                               boost::bind(&InterfaceInput::onConnect, this, _1),
			                                                               boost::bind(&InterfaceInput::onDisConnect, this, _1));
			//Set stereo output:
			std::vector<audio::channel> channelMap;
			channelMap.push_back(audio::channel_frontLeft);
			channelMap.push_back(audio::channel_frontRight);
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
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
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
		~InterfaceInput() {
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return;
			}
			m_interface->stop();
			m_interface.reset();
		}
		void onConnect(const ros::SingleSubscriberPublisher& _pub) {
			APPL_ERROR("on connect ... " << _pub.getSubscriberName());
		}
		void onDisConnect(const ros::SingleSubscriberPublisher& _pub) {
			APPL_ERROR("on dis-connect ... " << _pub.getSubscriberName());
		}
		void onDataReceived(const void* _data,
		                    const std11::chrono::system_clock::time_point& _time,
		                    size_t _nbChunk,
		                    enum audio::format _format,
		                    uint32_t _frequency,
		                    const std::vector<audio::channel>& _map) {
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
};

class InterfaceOutputStreamElement {
	private:
		int32_t m_id;
	public:
		int32_t getId() {
			return m_id;
		}
	private:
		int32_t m_nbConsecutiveUnderflow;
	public:
		int32_t getCountUnderflow() {
			return m_nbConsecutiveUnderflow;
		}
	private:
		std11::shared_ptr<river::Manager> m_manager;
		std11::shared_ptr<river::Interface> m_interface;
		std11::mutex m_mutex;
	public:
		InterfaceOutputStreamElement(const std11::shared_ptr<river::Manager>& _manager, int32_t _id) :
		  m_id(_id),
		  m_nbConsecutiveUnderflow(0),
		  m_manager(_manager) {
			APPL_INFO("Create interface");
		}
		~InterfaceOutputStreamElement() {
			std11::unique_lock<std::mutex> lock(m_mutex);
			APPL_INFO("Remove interfaces (start)");
			m_interface->stop();
			m_interface.reset();
			m_manager.reset();
			APPL_INFO("Remove interfaces (done)");
		}
		void onTopicMessage(const std::string& _streamName, const audio_msg::AudioBuffer::ConstPtr& _msg) {
			std11::unique_lock<std::mutex> lock(m_mutex);
			if (m_interface != nullptr) {
				APPL_VERBOSE("Write data : " << m_id << " size= " << _msg->data.size()/m_interface->getInterfaceFormat().getChunkSize());
				m_interface->write(&_msg->data[0], _msg->data.size()/m_interface->getInterfaceFormat().getChunkSize());
				m_nbConsecutiveUnderflow = 0;
				return;
			}
			audio::format format = audio::convertFormat(_msg->channelFormat);
			std::vector<enum audio::channel> map = audio::convertChannel(_msg->channelMap);
			// no interface found => create a new one
			m_interface = m_manager->createOutput(_msg->frequency,
			                                      map,
			                                      format,
			                                      _streamName);
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return;
			}
			m_interface->setReadwrite();
			m_interface->setStatusFunction(std11::bind(&InterfaceOutputStreamElement::onStatus, this, std11::placeholders::_1, std11::placeholders::_2, _msg->sourceId));
			m_interface->start();
			m_interface->write(&_msg->data[0], _msg->data.size()/m_interface->getInterfaceFormat().getChunkSize());
		}
		void onStatus(const std::string& _origin, const std::string& _status, int32_t _iii) {
			APPL_VERBOSE("status event : " << _origin << " status=" << _status << " on i=" << _iii);
			m_nbConsecutiveUnderflow++;
		}
};


class InterfaceOutputStreamManager {
	private:
		std::string m_name;
	public:
		const std::string& getName() {
			return m_name;
		}
	private:
		std11::shared_ptr<river::Manager> m_manager;
		std::vector<std11::shared_ptr<InterfaceOutputStreamElement> > m_elementList;
		std11::mutex m_mutex;
	public:
		InterfaceOutputStreamManager(const std::string& _name) :
		  m_name(_name) {
			m_manager = river::Manager::create(m_name);
			APPL_INFO("Create Manager : " << m_name);
		}
		~InterfaceOutputStreamManager() {
			APPL_INFO("Remove Manager : " << m_name);
			std11::unique_lock<std::mutex> lock(m_mutex);
			APPL_INFO("Clean list");
			m_elementList.clear();
			m_manager.reset();
			APPL_INFO("All is done ...");
		}
		void onTopicMessage(const std::string& _streamName, const audio_msg::AudioBuffer::ConstPtr& _msg) {
			std11::unique_lock<std::mutex> lock(m_mutex);
			for (size_t iii=0; iii<m_elementList.size(); ++iii) {
				if (m_elementList[iii] == nullptr) {
					continue;
				}
				if(m_elementList[iii]->getId() == _msg->sourceId) {
					m_elementList[iii]->onTopicMessage(_streamName, _msg);
					return;
				}
			}
			// no interface found => create a new one
			std11::shared_ptr<InterfaceOutputStreamElement> interface = std11::make_shared<InterfaceOutputStreamElement>(m_manager, _msg->sourceId);
			if(interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return;
			}
			m_elementList.push_back(interface);
			interface->onTopicMessage(_streamName, _msg);
			m_manager->generateDotAll("myDot.dot");
		}
		bool onTimer() {
			std::vector<std11::shared_ptr<InterfaceOutputStreamElement> >::iterator it = m_elementList.begin();
			bool oneElementRemoved = false;
			while (it != m_elementList.end()) {
				if (*it == nullptr) {
					it = m_elementList.erase(it);
					continue;
				}
				if ((*it)->getCountUnderflow() >= 50) {
					(*it).reset();
					oneElementRemoved = true;
					it = m_elementList.erase(it);
					continue;
				}
				++it;
			}
			if (oneElementRemoved == true) {
				m_manager->generateDotAll("myDot.dot");
			}
			// return remove ...
			return m_elementList.size() == 0;
		}
};

class InterfaceOutputStream {
	public:
		std::string m_lowLevelStreamName;
		ros::Subscriber m_stream;
		ros::Timer m_timer;
		std11::mutex m_mutex;
		std::vector<std11::shared_ptr<InterfaceOutputStreamManager> > m_list;
	public:
		InterfaceOutputStream(const std::string& _input="speaker", const std::string& _publisher="speaker") :
		  m_lowLevelStreamName("speaker") {
			ros::NodeHandle nodeHandlePrivate("~");
			m_stream = nodeHandlePrivate.subscribe<audio_msg::AudioBuffer>(_publisher,
			                                                               1000,
			                                                               boost::bind(&InterfaceOutputStream::onTopicMessage, this, _1));
			m_timer = nodeHandlePrivate.createTimer(ros::Duration(ros::Rate(4)), boost::bind(&InterfaceOutputStream::onTimer, this, _1));
		}
		~InterfaceOutputStream() {
			std11::unique_lock<std::mutex> lock(m_mutex);
			m_list.clear();
		}
		void onTopicMessage(const audio_msg::AudioBuffer::ConstPtr& _msg) {
			std11::unique_lock<std::mutex> lock(m_mutex);
			for (size_t iii=0; iii<m_list.size(); ++iii) {
				if (m_list[iii] == nullptr) {
					continue;
				}
				if (m_list[iii]->getName() == _msg->sourceName) {
					APPL_VERBOSE("Write data : " << _msg->sourceName);
					m_list[iii]->onTopicMessage(m_lowLevelStreamName, _msg);
					return;
				}
			}
			// new interface name:
			std11::shared_ptr<InterfaceOutputStreamManager> newInterface = std11::make_shared<InterfaceOutputStreamManager>(_msg->sourceName);
			if (newInterface == nullptr) {
				APPL_ERROR("can not generate new interface element...");
				return;
			}
			m_list.push_back(newInterface);
			newInterface->onTopicMessage(m_lowLevelStreamName, _msg);
		}
		void onTimer(const ros::TimerEvent& _timer) {
			std11::unique_lock<std::mutex> lock(m_mutex);
			std::vector<std11::shared_ptr<InterfaceOutputStreamManager> >::iterator it = m_list.begin();
			while (it != m_list.end()) {
				if (*it == nullptr) {
					it = m_list.erase(it);
					continue;
				}
				if ((*it)->onTimer() == true) {
					(*it).reset();
					it = m_list.erase(it);
					continue;
				}
				++it;
			}
		}
};


class InterfaceOutput {
	public:
		std11::shared_ptr<river::Manager> m_manager;
		std11::shared_ptr<river::Interface> m_interface;
		int64_t m_id;
	public:
		InterfaceOutput(std11::shared_ptr<river::Manager> _manager,
		                const std::string& _name,
		                enum audio::format _format,
		                uint32_t _frequency,
		                const std::vector<audio::channel>& _map) :
		  m_manager(_manager) {
			static int64_t uid = etk::tool::irand(0, 56000);
			m_id = uid++;
			uid += etk::tool::irand(0, 10);
			m_interface = m_manager->createOutput(_frequency,
			                                      _map,
			                                      _format,
			                                      "speaker");
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				m_id = -1;
				return;
			}
			m_interface->setReadwrite();
			m_interface->start();
		}
		~InterfaceOutput() {
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return;
			}
			m_interface->stop();
			m_interface.reset();
		}
		int64_t getId() {
			return m_id;
		}
		uint64_t write(const audio_core::writeRequest_<std::allocator<void> >::_data_type& _data) {
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return 1000000;
			}
			m_interface->write(&_data[0], _data.size()/m_interface->getInterfaceFormat().getChunkSize());
			// TODO : Do it better ...
			return m_interface->getBufferFillSizeMicrosecond().count()*4/5;
		}
		uint64_t getTimeBuffer() {
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return 0;
			}
			return m_interface->getBufferFillSizeMicrosecond().count();
		}
};

std11::shared_ptr<river::Manager> g_manager;
static std::vector<std11::shared_ptr<InterfaceOutput>> g_listInterafceOut;

bool f_create(audio_core::create::Request& _req,
              audio_core::create::Response& _res) {
	std11::shared_ptr<InterfaceOutput> newInterface;
	
	newInterface = std11::make_shared<InterfaceOutput>(g_manager,
	                                                   _req.flowName,
	                                                   audio::convertFormat(_req.channelFormat),
	                                                   _req.frequency,
	                                                   audio::convertChannel(_req.channelMap));
	if (newInterface == nullptr) {
		_res.handle = -1;
		return false;
	}
	_res.handle = newInterface->getId();
	mutex.lock();
	g_listInterafceOut.push_back(newInterface);
	mutex.unlock();
	APPL_INFO("create : [" << _res.handle << "] type: '" << _req.flowName << "'");
	return true;
}

bool f_remove(audio_core::remove::Request& _req,
              audio_core::remove::Response& _res) {
	std11::shared_ptr<InterfaceOutput> interface;
	mutex.lock();
	for(size_t iii=0; iii<g_listInterafceOut.size(); ++iii) {
		if (g_listInterafceOut[iii] == nullptr) {
			continue;
		}
		if (g_listInterafceOut[iii]->getId() == _req.handle) {
			interface = g_listInterafceOut[iii];
			// remove handle in the list
			g_listInterafceOut[iii].reset();
			g_listInterafceOut.erase(g_listInterafceOut.begin() + iii);
			break;
		}
	}
	mutex.unlock();
	if (interface == nullptr) {
		APPL_ERROR("remove : [" << _req.handle << "] Can not remove this ==> already removed.");
		return false;
	}
	// Remove interface :
	APPL_INFO("remove : [" << _req.handle << "] (start)");
	interface.reset();
	APPL_INFO("remove : [" << _req.handle << "] (end)");
	return true;
}

bool f_getBufferTime(audio_core::getBufferTime::Request& _req,
                     audio_core::getBufferTime::Response& _res) {
	std11::shared_ptr<InterfaceOutput> interface;
	// reset ouput
	_res.microseconds = 0;
	mutex.lock();
	// Find the handle:
	for(size_t iii=0; iii<g_listInterafceOut.size(); ++iii) {
		if (g_listInterafceOut[iii] == nullptr) {
			continue;
		}
		if (g_listInterafceOut[iii]->getId() == _req.handle) {
			interface = g_listInterafceOut[iii];
			break;
		}
	}
	mutex.unlock();
	if (interface == nullptr) {
		APPL_ERROR("getBufferTime : [" << _req.handle << "] Can not get time ==> handle does not exist...");
		return false;
	}
	_res.microseconds = interface->getTimeBuffer();
	return true;
}



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "audio_interface");
	etk::log::setLevel(etk::log::logLevelVerbose);
	etk::initDefaultFolder(_argv[0]);
	etk::setArgZero(_argv[0]);
	for (int32_t iii=0; iii<_argc; ++iii) {
		APPL_ERROR("Argument : " << iii << " '" << _argv[iii] << "'");
	}
	std::string hardwareInterface="DATA:hardware.json";
	for (int32_t iii=0; iii<_argc; ++iii) {
		std::string arg = _argv[iii];
		if (etk::start_with(_argv[iii], "--hardware=") == true) {
			hardwareInterface = std::string(&_argv[iii][11]);
			APPL_INFO("Find hardware configuration ... : '" << hardwareInterface << "'");
		}
	}
	river::init(hardwareInterface);
	
	ros::NodeHandle n;
	
	ros::ServiceServer serviceCreate = n.advertiseService("create", f_create);
	ros::ServiceServer serviceRemove = n.advertiseService("remove", f_remove);
	ros::ServiceServer serviceGetBufferTime = n.advertiseService("getBufferTime", f_getBufferTime);
	
	g_manager = river::Manager::create(n.getNamespace());
	// start publishing of Microphone
	std11::shared_ptr<InterfaceInput> m_input = std11::make_shared<InterfaceInput>(g_manager, "microphone", "microphone", false);
	// start publishing of Speaker feedback
	std11::shared_ptr<InterfaceInput> m_feedback = std11::make_shared<InterfaceInput>(g_manager, "speaker", "feedback/speaker", true);
	// create the Stream for output
	std11::shared_ptr<InterfaceOutputStream> m_speaker = std11::make_shared<InterfaceOutputStream>("speaker", "speaker");
	
	/*
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	ros::spin();
	m_input.reset();
	m_feedback.reset();
	g_manager.reset();
	river::unInit();
	
	return 0;
}
