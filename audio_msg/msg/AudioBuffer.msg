#################################
# enum: channel order properties:
#################################
uint8 CHANNEL_UNKNOW=0
uint8 CHANNEL_FRONT_LEFT=1
uint8 CHANNEL_FRONT_CENTER=2
uint8 CHANNEL_FRONT_RIGHT=3
uint8 CHANNEL_REAR_LEFT=4
uint8 CHANNEL_REAR_CENTER=5
uint8 CHANNEL_REAR_RIGHT=6
uint8 CHANNEL_SURROUND_LEFT=7
uint8 CHANNEL_SURROUND_RIGHT=8
uint8 CHANNEL_SUBWOOFER=9
uint8 CHANNEL_LFE=10

#######################
# enum: Channel Format:
#######################
uint8 FORMAT_UNKNOW=0
uint8 FORMAT_INT8=1
uint8 FORMAT_INT8_ON_INT16=2
uint8 FORMAT_INT16=3
uint8 FORMAT_INT16_ON_INT32=4
uint8 FORMAT_INT24=5
uint8 FORMAT_INT32=6
uint8 FORMAT_INT32_ON_INT64=7
uint8 FORMAT_INT64=8
uint8 FORMAT_FLOAT=9
uint8 FORMAT_DOUBLE=10

##########
# Message:
##########
# timestanp the audio buffer
Header header
# source
string sourceName
# source id
uint32 sourceId
# current frequency of the audio interface
uint16 frequency
# channel order of the current buffer
uint8[] channelMap
# Channel format of the data
uint8 channelFormat
# audio data buffer
uint8[] data