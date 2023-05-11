// Auto-generated. Do not edit!

// (in-package robotiq_3f_gripper_articulated_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Robotiq3FGripperRobotInput {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gACT = null;
      this.gMOD = null;
      this.gGTO = null;
      this.gIMC = null;
      this.gSTA = null;
      this.gDTA = null;
      this.gDTB = null;
      this.gDTC = null;
      this.gDTS = null;
      this.gFLT = null;
      this.gPRA = null;
      this.gPOA = null;
      this.gCUA = null;
      this.gPRB = null;
      this.gPOB = null;
      this.gCUB = null;
      this.gPRC = null;
      this.gPOC = null;
      this.gCUC = null;
      this.gPRS = null;
      this.gPOS = null;
      this.gCUS = null;
    }
    else {
      if (initObj.hasOwnProperty('gACT')) {
        this.gACT = initObj.gACT
      }
      else {
        this.gACT = 0;
      }
      if (initObj.hasOwnProperty('gMOD')) {
        this.gMOD = initObj.gMOD
      }
      else {
        this.gMOD = 0;
      }
      if (initObj.hasOwnProperty('gGTO')) {
        this.gGTO = initObj.gGTO
      }
      else {
        this.gGTO = 0;
      }
      if (initObj.hasOwnProperty('gIMC')) {
        this.gIMC = initObj.gIMC
      }
      else {
        this.gIMC = 0;
      }
      if (initObj.hasOwnProperty('gSTA')) {
        this.gSTA = initObj.gSTA
      }
      else {
        this.gSTA = 0;
      }
      if (initObj.hasOwnProperty('gDTA')) {
        this.gDTA = initObj.gDTA
      }
      else {
        this.gDTA = 0;
      }
      if (initObj.hasOwnProperty('gDTB')) {
        this.gDTB = initObj.gDTB
      }
      else {
        this.gDTB = 0;
      }
      if (initObj.hasOwnProperty('gDTC')) {
        this.gDTC = initObj.gDTC
      }
      else {
        this.gDTC = 0;
      }
      if (initObj.hasOwnProperty('gDTS')) {
        this.gDTS = initObj.gDTS
      }
      else {
        this.gDTS = 0;
      }
      if (initObj.hasOwnProperty('gFLT')) {
        this.gFLT = initObj.gFLT
      }
      else {
        this.gFLT = 0;
      }
      if (initObj.hasOwnProperty('gPRA')) {
        this.gPRA = initObj.gPRA
      }
      else {
        this.gPRA = 0;
      }
      if (initObj.hasOwnProperty('gPOA')) {
        this.gPOA = initObj.gPOA
      }
      else {
        this.gPOA = 0;
      }
      if (initObj.hasOwnProperty('gCUA')) {
        this.gCUA = initObj.gCUA
      }
      else {
        this.gCUA = 0;
      }
      if (initObj.hasOwnProperty('gPRB')) {
        this.gPRB = initObj.gPRB
      }
      else {
        this.gPRB = 0;
      }
      if (initObj.hasOwnProperty('gPOB')) {
        this.gPOB = initObj.gPOB
      }
      else {
        this.gPOB = 0;
      }
      if (initObj.hasOwnProperty('gCUB')) {
        this.gCUB = initObj.gCUB
      }
      else {
        this.gCUB = 0;
      }
      if (initObj.hasOwnProperty('gPRC')) {
        this.gPRC = initObj.gPRC
      }
      else {
        this.gPRC = 0;
      }
      if (initObj.hasOwnProperty('gPOC')) {
        this.gPOC = initObj.gPOC
      }
      else {
        this.gPOC = 0;
      }
      if (initObj.hasOwnProperty('gCUC')) {
        this.gCUC = initObj.gCUC
      }
      else {
        this.gCUC = 0;
      }
      if (initObj.hasOwnProperty('gPRS')) {
        this.gPRS = initObj.gPRS
      }
      else {
        this.gPRS = 0;
      }
      if (initObj.hasOwnProperty('gPOS')) {
        this.gPOS = initObj.gPOS
      }
      else {
        this.gPOS = 0;
      }
      if (initObj.hasOwnProperty('gCUS')) {
        this.gCUS = initObj.gCUS
      }
      else {
        this.gCUS = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robotiq3FGripperRobotInput
    // Serialize message field [gACT]
    bufferOffset = _serializer.uint8(obj.gACT, buffer, bufferOffset);
    // Serialize message field [gMOD]
    bufferOffset = _serializer.uint8(obj.gMOD, buffer, bufferOffset);
    // Serialize message field [gGTO]
    bufferOffset = _serializer.uint8(obj.gGTO, buffer, bufferOffset);
    // Serialize message field [gIMC]
    bufferOffset = _serializer.uint8(obj.gIMC, buffer, bufferOffset);
    // Serialize message field [gSTA]
    bufferOffset = _serializer.uint8(obj.gSTA, buffer, bufferOffset);
    // Serialize message field [gDTA]
    bufferOffset = _serializer.uint8(obj.gDTA, buffer, bufferOffset);
    // Serialize message field [gDTB]
    bufferOffset = _serializer.uint8(obj.gDTB, buffer, bufferOffset);
    // Serialize message field [gDTC]
    bufferOffset = _serializer.uint8(obj.gDTC, buffer, bufferOffset);
    // Serialize message field [gDTS]
    bufferOffset = _serializer.uint8(obj.gDTS, buffer, bufferOffset);
    // Serialize message field [gFLT]
    bufferOffset = _serializer.uint8(obj.gFLT, buffer, bufferOffset);
    // Serialize message field [gPRA]
    bufferOffset = _serializer.uint8(obj.gPRA, buffer, bufferOffset);
    // Serialize message field [gPOA]
    bufferOffset = _serializer.uint8(obj.gPOA, buffer, bufferOffset);
    // Serialize message field [gCUA]
    bufferOffset = _serializer.uint8(obj.gCUA, buffer, bufferOffset);
    // Serialize message field [gPRB]
    bufferOffset = _serializer.uint8(obj.gPRB, buffer, bufferOffset);
    // Serialize message field [gPOB]
    bufferOffset = _serializer.uint8(obj.gPOB, buffer, bufferOffset);
    // Serialize message field [gCUB]
    bufferOffset = _serializer.uint8(obj.gCUB, buffer, bufferOffset);
    // Serialize message field [gPRC]
    bufferOffset = _serializer.uint8(obj.gPRC, buffer, bufferOffset);
    // Serialize message field [gPOC]
    bufferOffset = _serializer.uint8(obj.gPOC, buffer, bufferOffset);
    // Serialize message field [gCUC]
    bufferOffset = _serializer.uint8(obj.gCUC, buffer, bufferOffset);
    // Serialize message field [gPRS]
    bufferOffset = _serializer.uint8(obj.gPRS, buffer, bufferOffset);
    // Serialize message field [gPOS]
    bufferOffset = _serializer.uint8(obj.gPOS, buffer, bufferOffset);
    // Serialize message field [gCUS]
    bufferOffset = _serializer.uint8(obj.gCUS, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robotiq3FGripperRobotInput
    let len;
    let data = new Robotiq3FGripperRobotInput(null);
    // Deserialize message field [gACT]
    data.gACT = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gMOD]
    data.gMOD = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gGTO]
    data.gGTO = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gIMC]
    data.gIMC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gSTA]
    data.gSTA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gDTA]
    data.gDTA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gDTB]
    data.gDTB = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gDTC]
    data.gDTC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gDTS]
    data.gDTS = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gFLT]
    data.gFLT = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPRA]
    data.gPRA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPOA]
    data.gPOA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gCUA]
    data.gCUA = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPRB]
    data.gPRB = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPOB]
    data.gPOB = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gCUB]
    data.gCUB = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPRC]
    data.gPRC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPOC]
    data.gPOC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gCUC]
    data.gCUC = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPRS]
    data.gPRS = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gPOS]
    data.gPOS = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gCUS]
    data.gCUS = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d0701156e580a420c48833f57bc83f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # gACT : Initialization status, echo of the rACT bit (activation bit).
    # 0x0 - Gripper reset.
    # 0x1 - Gripper activation.
    
    uint8 gACT
    
    # gMOD : Operation mode status, echo of the rMOD bits (grasping mode requested).
    # 0x00 - Basic mode.
    # 0x01 - Pinch mode.
    # 0x02 - Wide mode.
    # 0x03 - Scissor mode.
    
    uint8 gMOD
    
    # gGTO : Action status. echo of the rGTO bit (go to bit).
    # 0x0 - Stopped (or performing activation / grasping mode change / automatic release)
    # 0x1 - Go to Position Request
    
    uint8 gGTO
    
    # gIMC : Gripper status, returns the current status of the Gripper.
    # 0x00 - Gripper is in reset (or automatic release) state. See Fault status if Gripper is activated.
    # 0x01 - Activation is in progress.
    # 0x02 - Mode change is in progress.
    # 0x03 - Activation and mode change are completed.
    
    uint8 gIMC
    
    # gSTA : Motion status, returns the current motion of the Gripper fingers.
    # 0x00 - Gripper is in motion towards requested position (only meaningful if gGTO = 1)
    # 0x01 - Gripper is stopped. One or two fingers stopped before requested position
    # 0x02 - Gripper is stopped. All fingers stopped before requested position
    # 0x03 - Gripper is stopped. All fingers reached requested position
    
    uint8 gSTA
    
    # gDTA : Finger A object detection status returns information on possible object contact from finger A.
    # 0x00 - Finger A is in motion (only meaningful if gGTO = 1).
    # 0x01 - Finger A has stopped due to a contact while opening.
    # 0x02 - Finger A has stopped due to a contact while closing.
    # 0x03 - Finger A is at requested position.
    
    uint8 gDTA
    
    # gDTB : Finger B object detection status returns information on possible object contact from finger B.
    # 0x00 - Finger B is in motion (only meaningful if gGTO = 1).
    # 0x01 - Finger B has stopped due to a contact while opening.
    # 0x02 - Finger B has stopped due to a contact while closing.
    # 0x03 - Finger B is at requested position.
    
    uint8 gDTB
    
    # gDTC : Finger C object detection status returns information on possible object contact from finger A.
    # 0x00 - Finger C is in motion (only meaningful if gGTO = 1).
    # 0x01 - Finger C has stopped due to a contact while opening.
    # 0x02 - Finger C has stopped due to a contact while closing.
    # 0x03 - Finger C is at requested position.
    
    uint8 gDTC
    
    # gDTS : Scissor object detection status returns information on possible object contact from scissor.
    # 0x00 - Scissor is in motion (only meaningful if gGTO = 1).
    # 0x01 - Scissor has stopped due to a contact while opening.
    # 0x02 - Scissor has stopped due to a contact while closing.
    # 0x03 - Scissor is at requested position.
    
    uint8 gDTS
    
    # gFLT : Fault status returns general error messages useful for troubleshooting.
    #   0x00 - No fault (fault LED off)
    #   Priority faults (fault LED off)
    #     0x05 - Action delayed, activation (reactivation) must be completed prior to action.
    #     0x06 - Action delayed, mode change must be completed prior to action.
    #     0x07 - The activation bit must be set prior to action.
    #   Minor faults (fault LED continuous red)
    #     0x09 - The communication chip is not ready (may be booting).
    #     0x0A - Changing mode fault, interferences detected on Scissor (for less than 20 sec).
    #     0x0B - Automatic release in progress.
    #   Major faults (fault LED blinking red) - Reset is required
    #     0x0D - Activation fault, verify that no interference or other error occurred.
    #     0x0E - Changing mode fault, interferences detected on Scissor (for more than 20 sec).
    #     0x0F - Automatic release completed. Reset and activation is required.
    
    uint8 gFLT
    
    
    # gPRA : Echo of the requested position of the Gripper (rPRA),
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    # If commanding the Gripper in
    #   individual control mode, gPRA is the echo of finger A,
    #   otherwise it is the general position requested to all fingers.
    
    uint8 gPRA
    
    # gPOA : Returns the actual position of the Gripper finger A,
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    
    uint8 gPOA
    
    # gCUA : Returns a value that represents the finger A instantaneous current consumption from 0x00 to 0xFF.
    
    uint8 gCUA
    
    # gPRB : Echo of the requested position of finger B (rPRB),
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    
    uint8 gPRB
    
    # gPOB : Returns the actual position of the Gripper finger B,
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    
    uint8 gPOB
    
    # gCUB : Returns a value that represents the finger B instantaneous current consumption from 0x00 to 0xFF.
    
    uint8 gCUB
    
    # gPRC : Echo of the requested position of finger C (rPRC),
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    
    uint8 gPRC
    
    # gPOC : Returns the actual position of the Gripper finger C,
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    
    uint8 gPOC
    
    # gCUC : Returns a value that represents the finger C instantaneous current consumption from 0x00 to 0xFF.
    
    uint8 gCUC
    
    # gPRS : Echo of the requested position of scissor (rPRS),
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    
    uint8 gPRS
    
    # gPOS : Returns the actual position of the Gripper scissor,
    #   0x00 is minimum position (full opening) and
    #   0xFF is maximum position (full closing).
    
    uint8 gPOS
    
    # gCUS : Returns a value that represents the scissor instantaneous current consumption from 0x00 to 0xFF.
    #   0x00 is ...
    #   0xFF is ...
    
    uint8 gCUS
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robotiq3FGripperRobotInput(null);
    if (msg.gACT !== undefined) {
      resolved.gACT = msg.gACT;
    }
    else {
      resolved.gACT = 0
    }

    if (msg.gMOD !== undefined) {
      resolved.gMOD = msg.gMOD;
    }
    else {
      resolved.gMOD = 0
    }

    if (msg.gGTO !== undefined) {
      resolved.gGTO = msg.gGTO;
    }
    else {
      resolved.gGTO = 0
    }

    if (msg.gIMC !== undefined) {
      resolved.gIMC = msg.gIMC;
    }
    else {
      resolved.gIMC = 0
    }

    if (msg.gSTA !== undefined) {
      resolved.gSTA = msg.gSTA;
    }
    else {
      resolved.gSTA = 0
    }

    if (msg.gDTA !== undefined) {
      resolved.gDTA = msg.gDTA;
    }
    else {
      resolved.gDTA = 0
    }

    if (msg.gDTB !== undefined) {
      resolved.gDTB = msg.gDTB;
    }
    else {
      resolved.gDTB = 0
    }

    if (msg.gDTC !== undefined) {
      resolved.gDTC = msg.gDTC;
    }
    else {
      resolved.gDTC = 0
    }

    if (msg.gDTS !== undefined) {
      resolved.gDTS = msg.gDTS;
    }
    else {
      resolved.gDTS = 0
    }

    if (msg.gFLT !== undefined) {
      resolved.gFLT = msg.gFLT;
    }
    else {
      resolved.gFLT = 0
    }

    if (msg.gPRA !== undefined) {
      resolved.gPRA = msg.gPRA;
    }
    else {
      resolved.gPRA = 0
    }

    if (msg.gPOA !== undefined) {
      resolved.gPOA = msg.gPOA;
    }
    else {
      resolved.gPOA = 0
    }

    if (msg.gCUA !== undefined) {
      resolved.gCUA = msg.gCUA;
    }
    else {
      resolved.gCUA = 0
    }

    if (msg.gPRB !== undefined) {
      resolved.gPRB = msg.gPRB;
    }
    else {
      resolved.gPRB = 0
    }

    if (msg.gPOB !== undefined) {
      resolved.gPOB = msg.gPOB;
    }
    else {
      resolved.gPOB = 0
    }

    if (msg.gCUB !== undefined) {
      resolved.gCUB = msg.gCUB;
    }
    else {
      resolved.gCUB = 0
    }

    if (msg.gPRC !== undefined) {
      resolved.gPRC = msg.gPRC;
    }
    else {
      resolved.gPRC = 0
    }

    if (msg.gPOC !== undefined) {
      resolved.gPOC = msg.gPOC;
    }
    else {
      resolved.gPOC = 0
    }

    if (msg.gCUC !== undefined) {
      resolved.gCUC = msg.gCUC;
    }
    else {
      resolved.gCUC = 0
    }

    if (msg.gPRS !== undefined) {
      resolved.gPRS = msg.gPRS;
    }
    else {
      resolved.gPRS = 0
    }

    if (msg.gPOS !== undefined) {
      resolved.gPOS = msg.gPOS;
    }
    else {
      resolved.gPOS = 0
    }

    if (msg.gCUS !== undefined) {
      resolved.gCUS = msg.gCUS;
    }
    else {
      resolved.gCUS = 0
    }

    return resolved;
    }
};

module.exports = Robotiq3FGripperRobotInput;
