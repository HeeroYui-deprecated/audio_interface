#include "ros/ros.h"
#include "audio_msg/AudioBuffer.h"

#include <sstream>

FILE* filee = NULL;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void audioCallback(const audio_msg::AudioBuffer::ConstPtr& msg) {
	fwrite(&msg->data[0] , sizeof(int16_t), msg->data.size(), filee);
	ROS_INFO_STREAM("get message: freq=" << msg->frequency << " nbChannel=" << msg->channelMap.size() << " nbSample=" << msg->data.size()/msg->channelMap.size());
}

void usage() {
	ROS_INFO("test_audio_recorder usage [channel] [file]:");
	ROS_INFO("	[channel] Ros topic to get data");
	ROS_INFO("	[file] save file name (.raw only)");
	ROS_INFO("ex");
	ROS_INFO("	rosrun test_audio_recorder test_audio_recorder_node audio/microphone out.raw");
	exit (-1);
}

int main(int argc, char **argv) {
	if (argc != 3 ) {
		ROS_ERROR ("not enought argument : %d", argc);
		usage();
	}
	std::string p_channelToRecord = argv[1];
	std::string p_fileToSave = argv[2];
	
	filee = fopen(p_fileToSave.c_str(), "w");
	ros::init(argc, argv, "test_audio_recorder");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(p_channelToRecord, 1000, audioCallback);
	ros::spin();
	fclose(filee);
	return 0;
}

