// Auto-generated. Do not edit!

// (in-package franka_gripper_gazebo.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GripMsgRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.force = null;
    }
    else {
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripMsgRequest
    // Serialize message field [force]
    bufferOffset = _serializer.float64(obj.force, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripMsgRequest
    let len;
    let data = new GripMsgRequest(null);
    // Deserialize message field [force]
    data.force = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_gripper_gazebo/GripMsgRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e18a51329659ac6263f87aaf9a01fe62';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 force
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripMsgRequest(null);
    if (msg.force !== undefined) {
      resolved.force = msg.force;
    }
    else {
      resolved.force = 0.0
    }

    return resolved;
    }
};

class GripMsgResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripMsgResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripMsgResponse
    let len;
    let data = new GripMsgResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_gripper_gazebo/GripMsgResponse';
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
    const resolved = new GripMsgResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: GripMsgRequest,
  Response: GripMsgResponse,
  md5sum() { return 'e18a51329659ac6263f87aaf9a01fe62'; },
  datatype() { return 'franka_gripper_gazebo/GripMsg'; }
};
