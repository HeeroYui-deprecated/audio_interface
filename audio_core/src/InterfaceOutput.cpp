/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#include "debug.h"
#include "InterfaceOutput.h"

appl::InterfaceOutput::InterfaceOutput(const std::string& _input, const std::string& _publisher) :
  m_lowLevelStreamName(_input) {
	ros::NodeHandle nodeHandlePrivate("~");
	m_stream = nodeHandlePrivate.subscribe<audio_msg::AudioBuffer>(_publisher,
	                                                               1000,
	                                                               boost::bind(&InterfaceOutput::onTopicMessage, this, _1));
	m_timer = nodeHandlePrivate.createTimer(ros::Duration(ros::Rate(4)), boost::bind(&InterfaceOutput::onTimer, this, _1));
}

appl::InterfaceOutput::~InterfaceOutput() {
	std11::unique_lock<std::mutex> lock(m_mutex);
	m_list.clear();
}

void appl::InterfaceOutput::onTopicMessage(const audio_msg::AudioBuffer::ConstPtr& _msg) {
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
	std11::shared_ptr<appl::InterfaceOutputManager> newInterface = std11::make_shared<appl::InterfaceOutputManager>(_msg->sourceName);
	if (newInterface == nullptr) {
		APPL_ERROR("can not generate new interface element...");
		return;
	}
	m_list.push_back(newInterface);
	newInterface->onTopicMessage(m_lowLevelStreamName, _msg);
}

void appl::InterfaceOutput::onTimer(const ros::TimerEvent& _timer) {
	std11::unique_lock<std::mutex> lock(m_mutex);
	std::vector<std11::shared_ptr<appl::InterfaceOutputManager> >::iterator it = m_list.begin();
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
