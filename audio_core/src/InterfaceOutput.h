/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#ifndef __AUDIO_CORE_INTERFACE_OUTPUT_H__
#define __AUDIO_CORE_INTERFACE_OUTPUT_H__

#include "InterfaceOutputManager.h"
#include <ros/ros.h>
#include <audio_msg/AudioBuffer.h>

namespace appl {
	class InterfaceOutput {
		public:
			etk::String m_lowLevelStreamName;
			ros::Subscriber m_stream;
			ros::Timer m_timer;
			std11::mutex m_mutex;
			etk::Vector<std11::shared_ptr<appl::InterfaceOutputManager> > m_list;
		public:
			InterfaceOutput(const etk::String& _input="speaker", const etk::String& _publisher="speaker");
			~InterfaceOutput();
			void onTopicMessage(const audio_msg::AudioBuffer::ConstPtr& _msg);
			void onTimer(const ros::TimerEvent& _timer);
	};
}
#endif
