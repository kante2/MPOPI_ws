
"use strict";

let ManipulatorControl = require('./ManipulatorControl.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let DillyCmd = require('./DillyCmd.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let Obstacle = require('./Obstacle.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let ERP42Info = require('./ERP42Info.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let VehicleCollision = require('./VehicleCollision.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let ExternalForce = require('./ExternalForce.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let ShipState = require('./ShipState.js');
let WaitForTick = require('./WaitForTick.js');
let GhostMessage = require('./GhostMessage.js');
let IntersectionControl = require('./IntersectionControl.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let PREvent = require('./PREvent.js');
let Conveyor = require('./Conveyor.js');
let Obstacles = require('./Obstacles.js');
let SaveSensorData = require('./SaveSensorData.js');
let CollisionData = require('./CollisionData.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let CtrlCmd = require('./CtrlCmd.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let SensorPosControl = require('./SensorPosControl.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let MapSpec = require('./MapSpec.js');
let PRStatus = require('./PRStatus.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let ReplayInfo = require('./ReplayInfo.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let RadarDetection = require('./RadarDetection.js');
let RobotOutput = require('./RobotOutput.js');
let GPSMessage = require('./GPSMessage.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let VehicleSpec = require('./VehicleSpec.js');
let CMDConveyor = require('./CMDConveyor.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let EventInfo = require('./EventInfo.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let GVStateCmd = require('./GVStateCmd.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let TrafficLight = require('./TrafficLight.js');
let RadarDetections = require('./RadarDetections.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let Lamps = require('./Lamps.js');
let IntscnTL = require('./IntscnTL.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let WheelControl = require('./WheelControl.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let ObjectStatus = require('./ObjectStatus.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let Transforms = require('./Transforms.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let TOF = require('./TOF.js');
let VelocityCmd = require('./VelocityCmd.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let SVADC = require('./SVADC.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let RobotState = require('./RobotState.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');

module.exports = {
  ManipulatorControl: ManipulatorControl,
  SyncModeInfo: SyncModeInfo,
  GeoVector3Message: GeoVector3Message,
  VehicleSpecIndex: VehicleSpecIndex,
  WaitForTickResponse: WaitForTickResponse,
  MoraiTLIndex: MoraiTLIndex,
  DillyCmd: DillyCmd,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  Obstacle: Obstacle,
  SyncModeAddObject: SyncModeAddObject,
  MoraiSimProcStatus: MoraiSimProcStatus,
  ObjectStatusListExtended: ObjectStatusListExtended,
  ERP42Info: ERP42Info,
  MoraiSimProcHandle: MoraiSimProcHandle,
  WoowaDillyStatus: WoowaDillyStatus,
  VehicleCollision: VehicleCollision,
  ScenarioLoad: ScenarioLoad,
  ExternalForce: ExternalForce,
  IntersectionStatus: IntersectionStatus,
  ShipState: ShipState,
  WaitForTick: WaitForTick,
  GhostMessage: GhostMessage,
  IntersectionControl: IntersectionControl,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  PREvent: PREvent,
  Conveyor: Conveyor,
  Obstacles: Obstacles,
  SaveSensorData: SaveSensorData,
  CollisionData: CollisionData,
  DdCtrlCmd: DdCtrlCmd,
  CtrlCmd: CtrlCmd,
  ObjectStatusExtended: ObjectStatusExtended,
  FaultInjection_Response: FaultInjection_Response,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  ObjectStatusList: ObjectStatusList,
  SensorPosControl: SensorPosControl,
  EgoVehicleStatus: EgoVehicleStatus,
  NpcGhostCmd: NpcGhostCmd,
  MapSpec: MapSpec,
  PRStatus: PRStatus,
  FaultInjection_Controller: FaultInjection_Controller,
  FaultStatusInfo: FaultStatusInfo,
  ReplayInfo: ReplayInfo,
  MultiPlayEventRequest: MultiPlayEventRequest,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  RadarDetection: RadarDetection,
  RobotOutput: RobotOutput,
  GPSMessage: GPSMessage,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  VehicleSpec: VehicleSpec,
  CMDConveyor: CMDConveyor,
  SetTrafficLight: SetTrafficLight,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  SkateboardStatus: SkateboardStatus,
  EventInfo: EventInfo,
  SyncModeCmdResponse: SyncModeCmdResponse,
  SyncModeResultResponse: SyncModeResultResponse,
  DillyCmdResponse: DillyCmdResponse,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  GVStateCmd: GVStateCmd,
  NpcGhostInfo: NpcGhostInfo,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  GVDirectCmd: GVDirectCmd,
  SyncModeRemoveObject: SyncModeRemoveObject,
  TrafficLight: TrafficLight,
  RadarDetections: RadarDetections,
  FaultInjection_Sensor: FaultInjection_Sensor,
  FaultInjection_Tire: FaultInjection_Tire,
  Lamps: Lamps,
  IntscnTL: IntscnTL,
  MultiEgoSetting: MultiEgoSetting,
  WheelControl: WheelControl,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  SyncModeSetGear: SyncModeSetGear,
  MultiPlayEventResponse: MultiPlayEventResponse,
  ObjectStatus: ObjectStatus,
  MapSpecIndex: MapSpecIndex,
  MoraiSrvResponse: MoraiSrvResponse,
  ShipCtrlCmd: ShipCtrlCmd,
  Transforms: Transforms,
  MoraiTLInfo: MoraiTLInfo,
  PRCtrlCmd: PRCtrlCmd,
  TOF: TOF,
  VelocityCmd: VelocityCmd,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  SVADC: SVADC,
  SyncModeCmd: SyncModeCmd,
  RobotState: RobotState,
  VehicleCollisionData: VehicleCollisionData,
};
