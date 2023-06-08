
"use strict";

let GetSafetyMode = require('./GetSafetyMode.js')
let Popup = require('./Popup.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let Load = require('./Load.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetProgramState = require('./GetProgramState.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let RawRequest = require('./RawRequest.js')
let GetRobotMode = require('./GetRobotMode.js')
let AddToLog = require('./AddToLog.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')

module.exports = {
  GetSafetyMode: GetSafetyMode,
  Popup: Popup,
  IsProgramSaved: IsProgramSaved,
  Load: Load,
  GetLoadedProgram: GetLoadedProgram,
  GetProgramState: GetProgramState,
  IsProgramRunning: IsProgramRunning,
  RawRequest: RawRequest,
  GetRobotMode: GetRobotMode,
  AddToLog: AddToLog,
  IsInRemoteControl: IsInRemoteControl,
};
