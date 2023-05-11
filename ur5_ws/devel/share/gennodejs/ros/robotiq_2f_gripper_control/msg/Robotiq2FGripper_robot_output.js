// Auto-generated. Do not edit!

// (in-package robotiq_2f_gripper_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Robotiq2FGripper_robot_output {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rACT = null;
      this.rGTO = null;
      this.rATR = null;
      this.rPR = null;
      this.rSP = null;
      this.rFR = null;
    }
    else {
      if (initObj.hasOwnProperty('rACT')) {
        this.rACT = initObj.rACT
      }
      else {
        this.rACT = 0;
      }
      if (initObj.hasOwnProperty('rGTO')) {
        this.rGTO = initObj.rGTO
      }
      else {
        this.rGTO = 0;
      }
      if (initObj.hasOwnProperty('rATR')) {
        this.rATR = initObj.rATR
      }
      else {
        this.rATR = 0;
      }
      if (initObj.hasOwnProperty('rPR')) {
        this.rPR = initObj.rPR
      }
      else {
        this.rPR = 0;
      }
      if (initObj.hasOwnProperty('rSP')) {
        this.rSP = initObj.rSP
      }
      else {
        this.rSP = 0;
      }
      if (initObj.hasOwnProperty('rFR')) {
        this.rFR = initObj.rFR
      }
      else {
        this.rFR = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robotiq2FGripper_robot_output
    // Serialize message field [rACT]
    bufferOffset = _serializer.uint8(obj.rACT, buffer, bufferOffset);
    // Serialize message field [rGTO]
    bufferOffset = _serializer.uint8(obj.rGTO, buffer, bufferOffset);
    // Serialize message field [rATR]
    bufferOffset = _serializer.uint8(obj.rATR, buffer, bufferOffset);
    // Serialize message field [rPR]
    bufferOffset = _serializer.uint8(obj.rPR, buffer, bufferOffset);
    // Serialize message field [rSP]
    bufferOffset = _serializer.uint8(obj.rSP, buffer, bufferOffset);
    // Serialize message field [rFR]
    bufferOffset = _serializer.uint8(obj.rFR, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robotiq2FGripper_robot_output
    let len;
    let data = new Robotiq2FGripper_robot_output(null);
    // Deserialize message field [rACT]
    data.rACT = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rGTO]
    data.rGTO = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rATR]
    data.rATR = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rPR]
    data.rPR = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rSP]
    data.rSP = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rFR]
    data.rFR = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotiq_2f_gripper_control/Robotiq2FGripper_robot_output';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '481503a99d995d0e403b7ee708c6862c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 rACT
    uint8 rGTO
    uint8 rATR
    uint8 rPR
    uint8 rSP
    uint8 rFR
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robotiq2FGripper_robot_output(null);
    if (msg.rACT !== undefined) {
      resolved.rACT = msg.rACT;
    }
    else {
      resolved.rACT = 0
    }

    if (msg.rGTO !== undefined) {
      resolved.rGTO = msg.rGTO;
    }
    else {
      resolved.rGTO = 0
    }

    if (msg.rATR !== undefined) {
      resolved.rATR = msg.rATR;
    }
    else {
      resolved.rATR = 0
    }

    if (msg.rPR !== undefined) {
      resolved.rPR = msg.rPR;
    }
    else {
      resolved.rPR = 0
    }

    if (msg.rSP !== undefined) {
      resolved.rSP = msg.rSP;
    }
    else {
      resolved.rSP = 0
    }

    if (msg.rFR !== undefined) {
      resolved.rFR = msg.rFR;
    }
    else {
      resolved.rFR = 0
    }

    return resolved;
    }
};

module.exports = Robotiq2FGripper_robot_output;
