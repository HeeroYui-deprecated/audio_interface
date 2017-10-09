/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#ifndef __AUDIO_CORE_INTERFACE_OUTPUT_MANAGER_H__
#define __AUDIO_CORE_INTERFACE_OUTPUT_MANAGER_H__

#include "InterfaceOutputElement.h"
#include <ros/ros.h>
#include <audio_msg/AudioBuffer.h>

namespace appl {
	class InterfaceOutputManager {
		private:
			etk::String m_name;
		public:
			const etk::String& getName() {
				return m_name;
			}
		private:
			std11::shared_ptr<audio::river::Manager> m_manager;
			etk::Vector<std11::shared_ptr<appl::InterfaceOutputElement> > m_elementList;
			std11::mutex m_mutex;
		public:
			InterfaceOutputManager(const etk::String& _name);
			~InterfaceOutputManager();
			void onTopicMessage(const etk::String& _streamName, const audio_msg::AudioBuffer::ConstPtr& _msg);
			bool onTimer();
	};
}
#endif

