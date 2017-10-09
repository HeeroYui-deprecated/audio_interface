/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#include "debug.h"
#include "InterfaceOutputManager.h"

appl::InterfaceOutputManager::InterfaceOutputManager(const etk::String& _name) :
  m_name(_name) {
	m_manager = audio::river::Manager::create(m_name);
	APPL_INFO("Create Manager : " << m_name);
}

appl::InterfaceOutputManager::~InterfaceOutputManager() {
	APPL_INFO("Remove Manager : " << m_name);
	std11::unique_lock<ethread::Mutex> lock(m_mutex);
	APPL_INFO("Clean list");
	m_elementList.clear();
	m_manager.reset();
	APPL_INFO("All is done ...");
}

void appl::InterfaceOutputManager::onTopicMessage(const etk::String& _streamName, const audio_msg::AudioBuffer::ConstPtr& _msg) {
	std11::unique_lock<ethread::Mutex> lock(m_mutex);
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
	std11::shared_ptr<appl::InterfaceOutputElement> interface = std11::make_shared<appl::InterfaceOutputElement>(m_manager, _msg->sourceId);
	if(interface == nullptr) {
		APPL_ERROR("nullptr interface");
		return;
	}
	m_elementList.pushBack(interface);
	interface->onTopicMessage(_streamName, _msg);
	m_manager->generateDotAll("myDot.dot");
}

bool appl::InterfaceOutputManager::onTimer() {
	etk::Vector<std11::shared_ptr<appl::InterfaceOutputElement> >::iterator it = m_elementList.begin();
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
