// Auto-generated. Do not edit!

// (in-package moveit_planner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetVelocityRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.velScaling = null;
    }
    else {
      if (initObj.hasOwnProperty('velScaling')) {
        this.velScaling = initObj.velScaling
      }
      else {
        this.velScaling = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetVelocityRequest
    // Serialize message field [velScaling]
    bufferOffset = _serializer.float64(obj.velScaling, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetVelocityRequest
    let len;
    let data = new SetVelocityRequest(null);
    // Deserialize message field [velScaling]
    data.velScaling = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/SetVelocityRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a61a7a7956fba028d52c35ae7e4e38a7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 velScaling
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetVelocityRequest(null);
    if (msg.velScaling !== undefined) {
      resolved.velScaling = msg.velScaling;
    }
    else {
      resolved.velScaling = 0.0
    }

    return resolved;
    }
};

class SetVelocityResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetVelocityResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetVelocityResponse
    let len;
    let data = new SetVelocityResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/SetVelocityResponse';
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
    const resolved = new SetVelocityResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetVelocityRequest,
  Response: SetVelocityResponse,
  md5sum() { return 'a61a7a7956fba028d52c35ae7e4e38a7'; },
  datatype() { return 'moveit_planner/SetVelocity'; }
};
