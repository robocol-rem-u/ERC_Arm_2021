
"use strict";

let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let IOStates = require('./IOStates.js');
let Analog = require('./Analog.js');
let RobotModeDataMsg = require('./RobotModeDataMsg.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let Digital = require('./Digital.js');

module.exports = {
  MasterboardDataMsg: MasterboardDataMsg,
  IOStates: IOStates,
  Analog: Analog,
  RobotModeDataMsg: RobotModeDataMsg,
  ToolDataMsg: ToolDataMsg,
  RobotStateRTMsg: RobotStateRTMsg,
  Digital: Digital,
};
