/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#include "debug.h"
#include "InterfaceOutputElement.h"

appl::InterfaceOutputElement::InterfaceOutputElement(const std11::shared_ptr<audio::river::Manager>& _manager, int32_t _id) :
  m_id(_id),
  m_nbConsecutiveUnderflow(0),
  m_manager(_manager) {
	APPL_INFO("Create interface");
}

appl::InterfaceOutputElement::~InterfaceOutputElement() {
	std11::unique_lock<ethread::Mutex> lock(m_mutex);
	APPL_INFO("Remove interfaces (start)");
	m_interface->stop();
	m_interface.reset();
	m_manager.reset();
	APPL_INFO("Remove interfaces (done)");
}

void appl::InterfaceOutputElement::onTopicMessage(const etk::String& _streamName, const audio_msg::AudioBuffer::ConstPtr& _msg) {
	std11::unique_lock<ethread::Mutex> lock(m_mutex);
	if (m_interface != nullptr) {
		APPL_VERBOSE("Write data : " << m_id << " size= " << _msg->data.size()/m_interface->getInterfaceFormat().getChunkSize());
		m_interface->write(&_msg->data[0], _msg->data.size()/m_interface->getInterfaceFormat().getChunkSize());
		m_nbConsecutiveUnderflow = 0;
		return;
	}
	audio::format format = audio::convertFormat(_msg->channelFormat);
	etk::Vector<enum audio::channel> map = audio::convertChannel(_msg->channelMap);
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
	m_interface->setStatusFunction(std11::bind(&InterfaceOutputElement::onStatus, this, std11::placeholders::_1, std11::placeholders::_2, _msg->sourceId));
	m_interface->start();
	m_interface->write(&_msg->data[0], _msg->data.size()/m_interface->getInterfaceFormat().getChunkSize());
}

void appl::InterfaceOutputElement::onStatus(const etk::String& _origin, const etk::String& _status, int32_t _iii) {
	APPL_VERBOSE("status event : " << _origin << " status=" << _status << " on i=" << _iii);
	m_nbConsecutiveUnderflow++;
}
