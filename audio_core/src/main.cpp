

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

ros::Time startTime;
/*
static const double delayTimeBeforRemoveOutput = 30.0; // 30 second delay befor removing output that not create sound.

class IOPorperty {
	public:
		float frequencyIn;
		float frequencyOut;
		std::vector<uint8_t> inputChannelMap;
		std::vector<uint8_t> outputChannelMap;
};
IOPorperty audioProperties;


class InterfacePorperty {
	public:
		int64_t uid;
		std::string descriptiveString;
		float frequency;
		std::vector<uint8_t> channelMap;
		std::vector<int16_t> data;
		double playTime;
		double requestTime;
		InterfacePorperty():
		  uid(0),
		  frequency(0),
		  playTime(0),
		  requestTime(0) {
			static int64_t suid = 25;
			uid = suid++;
		}
};

std::vector<InterfacePorperty> listFlow;
*/
std11::mutex mutex;
/*
// RT audio input callback
int inoutInput(void* _outputBuffer,
               void* _inputBuffer,
               unsigned int _nBufferFrames,
               double _streamTime,
               RtAudioStreamStatus _status,
               void* _data) {
	IOPorperty* audioProperty = (IOPorperty*)_data;
	// Since the number of input and output channels is equal, we can do
	// a simple buffer copy operation here.
	if (_status) {
		ROS_ERROR("Stream overflow detected.");
	}
	// check if someone suscribe to the output:
	if (stream_microphone.getNumSubscribers() > 0) {
		//ROS_INFO("Publish data ... %d frames time %lf", _nBufferFrames, _streamTime);
		audio_msg::AudioBuffer msg;
		// create the Ros timestamp
		msg.header.stamp = startTime + ros::Duration(_streamTime, (_streamTime-(int32_t)_streamTime)*1000000000.0);
		// set message frequency
		msg.frequency = audioProperty->frequencyIn;
		// set channel mas properties
		msg.channelMap = audioProperty->inputChannelMap;
		// copy data:
		msg.data.resize(_nBufferFrames*audioProperty->inputChannelMap.size());
		memcpy(&msg.data[0], _inputBuffer, _nBufferFrames*audioProperty->inputChannelMap.size()*sizeof(int16_t));
		// publish message
		stream_microphone.publish(msg);
	}
	return 0;
}

// RT audio out callback
int inoutOutput(void* _outputBuffer,
                void* _inputBuffer,
                unsigned int _nBufferFrames,
                double _streamTime,
                RtAudioStreamStatus _status,
                void* _data) {
	IOPorperty* audioProperty = (IOPorperty*)_data;
	// Since the number of input and output channels is equal, we can do
	// a simple buffer copy operation here.
	if (_status) {
		ROS_ERROR("Stream underflow detected.");
	}
	std::vector<int32_t> output;
	int32_t newsize = (int64_t)audioProperty->outputChannelMap.size()*(int64_t)_nBufferFrames;
	//ROS_INFO("read %d*%d=%d ", audioProperty->outputChannelMap.size(), _nBufferFrames, newsize);
	output.resize(newsize, 0);
	mutex.lock();
	std::vector<InterfacePorperty>::iterator it = listFlow.begin();
	while (it != listFlow.end()) {
		// copy data...
		size_t availlable = it->data.size();
		availlable = availlable<output.size()?availlable:output.size();
		if (availlable > 0) {
			for (size_t iii=0; iii<availlable; ++iii) {
				output[iii] += it->data[iii];
			}
			//ROS_INFO("write %ld (better have : %ld)", availlable, output.size());
			it->data.erase(it->data.begin(), it->data.begin()+availlable);
			it->playTime = _streamTime;
		} else {
			ROS_INFO("[%d] underflow %d", (int)it->uid, (int)output.size());
			if (it->playTime <= 0) {
				double delay = it->playTime + _streamTime;
				if (delay > delayTimeBeforRemoveOutput) {
					ROS_ERROR("[%d] underflow ==> too much : Remove interface (after %lf s)", (int)it->uid, delay);
					it = listFlow.erase(it);
					continue;
				}
			} else {
				it->playTime = -_streamTime;
			}
		}
		++it;
	}
	mutex.unlock();
	if (_outputBuffer == NULL) {
		ROS_ERROR("NULL output");
		return 0;
	}
	int16_t* outputPointer = (int16_t*)_outputBuffer;
	for (size_t iii=0; iii<output.size(); ++iii) {
		*outputPointer++ = output[iii]>32767?32767:(output[iii]<-32767?-32767:(int16_t)output[iii]);
	}
	// check if someone is connected
	if (stream_speaker.getNumSubscribers() > 0) {
		//ROS_INFO("Get data ... %d frames time %lf", _nBufferFrames, _streamTime);
		audio_msg::AudioBuffer msg;
		// Create the ROS message time
		msg.header.stamp = startTime + ros::Duration(_streamTime, (_streamTime-(int32_t)_streamTime)*1000000000.0);
		// set message frequency
		msg.frequency = audioProperty->frequencyOut;
		// set channel mas properties
		msg.channelMap = audioProperty->outputChannelMap;
		// copy data:
		msg.data.resize(_nBufferFrames*audioProperty->outputChannelMap.size());
		memcpy(&msg.data[0], _outputBuffer, _nBufferFrames*audioProperty->outputChannelMap.size()*sizeof(int16_t));
		// publish message
		stream_speaker.publish(msg);
	}
	return 0;
}
*/

bool f_create(audio_core::create::Request &req,
              audio_core::create::Response &res) {
	/*
	InterfacePorperty newInterface;
	newInterface.descriptiveString = req.typeFlow;
	newInterface.frequency = req.frequency;
	newInterface.channelMap = req.channelMap;
	double requestTime = (double)req.stamp.sec + (double)req.stamp.nsec*0.0000000001;
	newInterface.requestTime = requestTime;
	newInterface.playTime = 0.0;
	res.handle = newInterface.uid;
	mutex.lock();
	listFlow.push_back(newInterface);
	mutex.unlock();
	ROS_INFO("create : [%d] type : %s", (int)res.handle, req.typeFlow.c_str());
	*/
	return true;
}

bool f_remove(audio_core::remove::Request &req,
              audio_core::remove::Response &res) {
	/*
	mutex.lock();
	std::vector<InterfacePorperty>::iterator it = listFlow.begin();
	while (it != listFlow.end()) {
		if (it->uid == req.handle) {
			it = listFlow.erase(it);
		} else {
			++it;
		}
	}
	mutex.unlock();
	ROS_INFO("remove : [%d]", (int)req.handle);
	*/
	return true;
}


bool f_write(audio_core::write::Request &req,
             audio_core::write::Response &res) {
	//ROS_INFO("write : [%ld] nbSample=%d", req.handle, (int32_t)req.data.size());
	/*
	mutex.lock();
	std::vector<InterfacePorperty>::iterator it = listFlow.begin();
	while (it != listFlow.end()) {
		if (it->uid == req.handle) {
			std::vector<int16_t> data = convert(it->channelMap, it->frequency, req.data, audioProperties.outputChannelMap, audioProperties.frequencyOut);
			size_t oldSize = it->data.size();
			it->data.resize(oldSize + data.size());
			memcpy(&it->data[oldSize], &data[0], data.size()*sizeof(int16_t));
			res.waitTime = (float)(it->data.size() / audioProperties.outputChannelMap.size()) / (float)audioProperties.frequencyOut * 1000000.0f;
			if (res.waitTime > 200000) {
				res.waitTime -= 200000;
			} else {
				res.waitTime = 0;
			}
			break;
		}
		++it;
	}
	mutex.unlock();
	*/
	return true;
}

bool f_getBufferTime(audio_core::getBufferTime::Request &req,
                     audio_core::getBufferTime::Response &res) {
	//ROS_INFO("get Time: [%ld]", req.handle, (int32_t)req.data.size());
	/*
	mutex.lock();
	std::vector<InterfacePorperty>::iterator it = listFlow.begin();
	while (it != listFlow.end()) {
		if (it->uid == req.handle) {
			res.time = (float)(it->data.size() / audioProperties.outputChannelMap.size()) / (float)audioProperties.frequencyOut * 1000000.0f;
			break;
		}
		++it;
	}
	mutex.unlock();
	*/
	return true;
}
/*
	output.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
	output.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
	output.push_back(audio_msg::AudioBuffer::CHANNEL_REAR_LEFT);
	output.push_back(audio_msg::AudioBuffer::CHANNEL_REAR_RIGHT);
	output.push_back(audio_msg::AudioBuffer::CHANNEL_SURROUND_LEFT);
	output.push_back(audio_msg::AudioBuffer::CHANNEL_SURROUND_RIGHT);
	output.push_back(audio_msg::AudioBuffer::CHANNEL_LFE);
	output.push_back(audio_msg::AudioBuffer::CHANNEL_SUBWOOFER);
*/



class InterfaceOutput {
	public:
		std::vector<audio::channel> m_channelMap;
		std11::shared_ptr<river::Manager> m_manager;
		std11::shared_ptr<river::Interface> m_interface;
	public:
		InterfaceOutput(std11::shared_ptr<river::Manager> _manager) :
		  m_manager(_manager) {
			//Set stereo output:
			m_channelMap.push_back(audio::channel_frontLeft);
			m_channelMap.push_back(audio::channel_frontRight);
			m_interface = m_manager->createOutput(48000,
			                                      m_channelMap,
			                                      audio::format_int16,
			                                      "speaker");
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return;
			}
			m_interface->setReadwrite();
		}
		void start() {
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return;
			}
			m_interface->start();
		}
		void stop() {
			//m_interface->write(&data[0], data.size()/m_channelMap.size());
			m_interface->stop();
		}
};


class InterfaceInput {
	public:
		std11::shared_ptr<river::Manager> m_manager;
		std11::shared_ptr<river::Interface> m_interface;
		ros::Publisher m_stream;
		std11::mutex mutex;
	public:
		InterfaceInput(std11::shared_ptr<river::Manager> _manager, const std::string& _input="microphone", const std::string& _publisher="microphone") :
		  m_manager(_manager) {
			ros::NodeHandle nodeHandlePrivate("~");
			m_stream = nodeHandlePrivate.advertise<audio_msg::AudioBuffer>(_publisher, 100);
			//Set stereo output:
			std::vector<audio::channel> channelMap;
			channelMap.push_back(audio::channel_frontLeft);
			channelMap.push_back(audio::channel_frontRight);
			m_interface = m_manager->createInput(48000,
			                                     channelMap,
			                                     audio::format_int16,
			                                     _input);
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
				//ROS_INFO("Publish data ... %d frames time %lf", _nBufferFrames, _streamTime);
				audio_msg::AudioBuffer msg;
				// create the Ros timestamp
				msg.header.stamp = ros::Time::now();//_time;
				// set message frequency
				msg.frequency = _frequency;
				// set channel mas properties
				//msg.channelMap = audioProperty->inputChannelMap;
				msg.channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_LEFT);
				msg.channelMap.push_back(audio_msg::AudioBuffer::CHANNEL_FRONT_RIGHT);
				// copy data:
				msg.data.resize(_nbChunk*_map.size());
				memcpy(&msg.data[0], _data, _nbChunk*_map.size()*sizeof(int16_t));
				// publish message
				m_stream.publish(msg);
			}
		}
		void start() {
			if(m_interface == nullptr) {
				APPL_ERROR("nullptr interface");
				return;
			}
			// wait 2 second ...
			usleep(20000000);
			m_interface->stop();
		}
};



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
	ros::ServiceServer serviceWrite = n.advertiseService("write", f_write);
	ros::ServiceServer serviceGetBufferTime = n.advertiseService("getBufferTime", f_getBufferTime);
	
	ros::NodeHandle nodeHandlePrivate("~");
	
	std11::shared_ptr<river::Manager> m_manager = river::Manager::create("ROS node");
	
	std11::shared_ptr<InterfaceInput> m_input = std11::make_shared<InterfaceInput>(m_manager, "microphone", "mic");
	
	/*
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	ros::spin();
	m_input.reset();
	m_manager.reset();
	river::unInit();
	
	return 0;
}
