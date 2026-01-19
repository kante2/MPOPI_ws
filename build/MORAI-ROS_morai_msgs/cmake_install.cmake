# Install script for directory: /home/autonav/aim_ws/src/MORAI-ROS_morai_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/autonav/aim_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/msg" TYPE FILE FILES
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/CtrlCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/EgoVehicleStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/EgoVehicleStatusExtended.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/GPSMessage.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/GhostMessage.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ObjectStatusList.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ObjectStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ObjectStatusExtended.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ObjectStatusListExtended.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/TrafficLight.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ERP42Info.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/GetTrafficLightStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SetTrafficLight.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/IntersectionControl.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/IntersectionStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/CollisionData.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MultiEgoSetting.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/IntscnTL.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SensorPosControl.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MoraiSimProcHandle.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MoraiSimProcStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MoraiSrvResponse.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ScenarioLoad.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MoraiTLIndex.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MoraiTLInfo.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SaveSensorData.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ReplayInfo.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/EventInfo.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/Lamps.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/VehicleSpec.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/VehicleSpecIndex.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/NpcGhostCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/NpcGhostInfo.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/VehicleCollisionData.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/VehicleCollision.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeAddObject.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeInfo.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/WaitForTickResponse.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeRemoveObject.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeCmdResponse.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/WaitForTick.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MapSpec.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MapSpecIndex.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeCtrlCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeSetGear.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeResultResponse.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SyncModeScenarioLoad.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/RadarDetection.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/RadarDetections.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/PRStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/PRCtrlCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/PREvent.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SkateboardCtrlCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SkateboardStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SkidSteer6wUGVCtrlCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SkidSteer6wUGVStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MultiPlayEventResponse.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/MultiPlayEventRequest.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/DillyCmdResponse.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/DillyCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/WoowaDillyStatus.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/SVADC.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultInjection_Controller.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultInjection_Response.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultInjection_Sensor.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultInjection_Tire.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultStatusInfo_Overall.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultStatusInfo_Sensor.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultStatusInfo_Vehicle.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/FaultStatusInfo.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/UGVServeSkidCtrlCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/VelocityCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/Obstacle.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/Obstacles.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/Transforms.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/GVDirectCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/GVStateCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/TOF.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/RobotOutput.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/WheelControl.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/RobotState.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/Conveyor.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/CMDConveyor.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ExternalForce.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/GeoVector3Message.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ShipState.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ShipCtrlCmd.msg"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/msg/ManipulatorControl.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/srv" TYPE FILE FILES
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiScenarioLoadSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiSimProcSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiTLInfoSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiEventCmdSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiVehicleSpecSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiSyncModeCmdSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiWaitForTickSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiMapSpecSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiSyncModeCtrlCmdSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiSyncModeSetGearSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiSyncModeSLSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/PREventSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiSyncModeAddObjectSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MoraiSyncModeRemoveObjectSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/MultiPlayEventSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/WoowaDillyEventCmdSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/FaultInjectionCtrlSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/FaultInjectionSensorSrv.srv"
    "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/srv/FaultInjectionTireSrv.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/cmake" TYPE FILE FILES "/home/autonav/aim_ws/build/MORAI-ROS_morai_msgs/catkin_generated/installspace/morai_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/autonav/aim_ws/devel/include/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/autonav/aim_ws/devel/share/roseus/ros/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/autonav/aim_ws/devel/share/common-lisp/ros/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/autonav/aim_ws/devel/share/gennodejs/ros/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/autonav/aim_ws/devel/lib/python3/dist-packages/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/autonav/aim_ws/devel/lib/python3/dist-packages/morai_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/autonav/aim_ws/build/MORAI-ROS_morai_msgs/catkin_generated/installspace/morai_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/cmake" TYPE FILE FILES "/home/autonav/aim_ws/build/MORAI-ROS_morai_msgs/catkin_generated/installspace/morai_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs/cmake" TYPE FILE FILES
    "/home/autonav/aim_ws/build/MORAI-ROS_morai_msgs/catkin_generated/installspace/morai_msgsConfig.cmake"
    "/home/autonav/aim_ws/build/MORAI-ROS_morai_msgs/catkin_generated/installspace/morai_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/morai_msgs" TYPE FILE FILES "/home/autonav/aim_ws/src/MORAI-ROS_morai_msgs/package.xml")
endif()

