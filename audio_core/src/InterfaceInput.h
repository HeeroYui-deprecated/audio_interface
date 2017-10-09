/**
 * @author Edouard DUPIN
 * 
 * @copyright 2015, Edouard DUPIN, all right reserved
 * 
 * @license APACHE v2.0 (see license file)
 */

#ifndef __AUDIO_CORE_INTERFACE_INPUT_H__
#define __AUDIO_CORE_INTERFACE_INPUT_H__

#include <audio/river/Manager.h>

namespace appl {
	class InterfaceInput {
		public:
			std11::shared_ptr<audio::river::Manager> m_manager;
			std11::shared_ptr<audio::river::Interface> m_interface;
			ros::Publisher m_stream;
			std11::mutex mutex;
		public:
			InterfaceInput(std11::shared_ptr<audio::river::Manager> _manager,
			               const etk::String& _input="microphone",
			               const etk::String& _publisher="microphone",
			               bool _feedback=false);
			~InterfaceInput();
		protected:
			void onConnect(const ros::SingleSubscriberPublisher& _pub);
			void onDisConnect(const ros::SingleSubscriberPublisher& _pub);
			void onDataReceived(const void* _data,
			                    const audio::Time& _time,
			                    size_t _nbChunk,
			                    enum audio::format _format,
			                    uint32_t _frequency,
			                    const etk::Vector<audio::channel>& _map);
	};
}

#endif
