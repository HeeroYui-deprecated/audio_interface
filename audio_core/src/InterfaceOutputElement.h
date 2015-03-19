/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#ifndef __AUDIO_CORE_INTERFACE_OUTPUT_ELEMENT_H__
#define __AUDIO_CORE_INTERFACE_OUTPUT_ELEMENT_H__

#include <river/Manager.h>
#include <river/Interface.h>
#include <ros/ros.h>
#include <audio_msg/AudioBuffer.h>

namespace appl {
	class InterfaceOutputElement {
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
			InterfaceOutputElement(const std11::shared_ptr<river::Manager>& _manager, int32_t _id);
			~InterfaceOutputElement();
			void onTopicMessage(const std::string& _streamName, const audio_msg::AudioBuffer::ConstPtr& _msg);
			void onStatus(const std::string& _origin, const std::string& _status, int32_t _iii);
	};
}

#endif
