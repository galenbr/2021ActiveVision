// Auto-generated. Do not edit!

// (in-package netft_utils.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class StartSimRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.simDim = null;
      this.simType = null;
      this.simSlope = null;
      this.maxForce = null;
    }
    else {
      if (initObj.hasOwnProperty('simDim')) {
        this.simDim = initObj.simDim
      }
      else {
        this.simDim = 0;
      }
      if (initObj.hasOwnProperty('simType')) {
        this.simType = initObj.simType
      }
      else {
        this.simType = 0;
      }
      if (initObj.hasOwnProperty('simSlope')) {
        this.simSlope = initObj.simSlope
      }
      else {
        this.simSlope = 0.0;
      }
      if (initObj.hasOwnProperty('maxForce')) {
        this.maxForce = initObj.maxForce
      }
      else {
        this.maxForce = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartSimRequest
    // Serialize message field [simDim]
    bufferOffset = _serializer.int32(obj.simDim, buffer, bufferOffset);
    // Serialize message field [simType]
    bufferOffset = _serializer.int32(obj.simType, buffer, bufferOffset);
    // Serialize message field [simSlope]
    bufferOffset = _serializer.float64(obj.simSlope, buffer, bufferOffset);
    // Serialize message field [maxForce]
    bufferOffset = _serializer.float64(obj.maxForce, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartSimRequest
    let len;
    let data = new StartSimRequest(null);
    // Deserialize message field [simDim]
    data.simDim = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [simType]
    data.simType = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [simSlope]
    data.simSlope = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [maxForce]
    data.maxForce = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/StartSimRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9bc0ea45e890996bb2e2ce115e7c8c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 simDim
    int32 simType
    float64 simSlope
    float64 maxForce
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartSimRequest(null);
    if (msg.simDim !== undefined) {
      resolved.simDim = msg.simDim;
    }
    else {
      resolved.simDim = 0
    }

    if (msg.simType !== undefined) {
      resolved.simType = msg.simType;
    }
    else {
      resolved.simType = 0
    }

    if (msg.simSlope !== undefined) {
      resolved.simSlope = msg.simSlope;
    }
    else {
      resolved.simSlope = 0.0
    }

    if (msg.maxForce !== undefined) {
      resolved.maxForce = msg.maxForce;
    }
    else {
      resolved.maxForce = 0.0
    }

    return resolved;
    }
};

class StartSimResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartSimResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartSimResponse
    let len;
    let data = new StartSimResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/StartSimResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartSimResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: StartSimRequest,
  Response: StartSimResponse,
  md5sum() { return 'f9bc0ea45e890996bb2e2ce115e7c8c3'; },
  datatype() { return 'netft_utils/StartSim'; }
};
