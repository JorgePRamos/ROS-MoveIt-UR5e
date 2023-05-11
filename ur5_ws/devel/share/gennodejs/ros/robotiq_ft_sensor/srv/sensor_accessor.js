// Auto-generated. Do not edit!

// (in-package robotiq_ft_sensor.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class sensor_accessorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command_id = null;
      this.command = null;
    }
    else {
      if (initObj.hasOwnProperty('command_id')) {
        this.command_id = initObj.command_id
      }
      else {
        this.command_id = 0;
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sensor_accessorRequest
    // Serialize message field [command_id]
    bufferOffset = _serializer.uint8(obj.command_id, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sensor_accessorRequest
    let len;
    let data = new sensor_accessorRequest(null);
    // Deserialize message field [command_id]
    data.command_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.command);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotiq_ft_sensor/sensor_accessorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '863e7f766b688715a9cf50b105733d81';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 COMMAND_GET_SERIAL_NUMBER=1
    uint8 COMMAND_GET_FIRMWARE_VERSION=2
    uint8 COMMAND_GET_PRODUCTION_YEAR=4
    uint8 COMMAND_SET_ZERO=8
    uint8 command_id
    string command  # deprecated, please use command_id with a value of COMMAND_*
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sensor_accessorRequest(null);
    if (msg.command_id !== undefined) {
      resolved.command_id = msg.command_id;
    }
    else {
      resolved.command_id = 0
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    return resolved;
    }
};

// Constants for message
sensor_accessorRequest.Constants = {
  COMMAND_GET_SERIAL_NUMBER: 1,
  COMMAND_GET_FIRMWARE_VERSION: 2,
  COMMAND_GET_PRODUCTION_YEAR: 4,
  COMMAND_SET_ZERO: 8,
}

class sensor_accessorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.res = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('res')) {
        this.res = initObj.res
      }
      else {
        this.res = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sensor_accessorResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [res]
    bufferOffset = _serializer.string(obj.res, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sensor_accessorResponse
    let len;
    let data = new sensor_accessorResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [res]
    data.res = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.res);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robotiq_ft_sensor/sensor_accessorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2adcefa00ba94fe7b359ee9018245fbf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string res
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sensor_accessorResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.res !== undefined) {
      resolved.res = msg.res;
    }
    else {
      resolved.res = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: sensor_accessorRequest,
  Response: sensor_accessorResponse,
  md5sum() { return 'c8c699ce907261a9c404410e16c0ef0d'; },
  datatype() { return 'robotiq_ft_sensor/sensor_accessor'; }
};
