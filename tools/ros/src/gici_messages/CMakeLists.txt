cmake_minimum_required(VERSION 2.8)
project(gici_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  message_generation
)

add_message_files(
  FILES
  GlonassEphemeris.msg
  GnssAntennaPosition.msg
  GnssEphemerides.msg
  GnssEphemeris.msg
  GnssIonosphereParameter.msg
  GnssObservation.msg
  GnssObservations.msg
  GnssSsrCodeBias.msg
  GnssSsrCodeBiases.msg
  GnssSsrEphemerides.msg
  GnssSsrEphemeris.msg
  GnssSsrPhaseBias.msg
  GnssSsrPhaseBiases.msg
) 

generate_messages(
  DEPENDENCIES std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp std_msgs message_generation
)
