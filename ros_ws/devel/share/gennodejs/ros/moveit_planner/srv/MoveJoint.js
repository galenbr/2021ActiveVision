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

class MoveJointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.val = null;
      this.execute = null;
    }
    else {
      if (initObj.hasOwnProperty('val')) {
        this.val = initObj.val
      }
      else {
        this.val = [];
      }
      if (initObj.hasOwnProperty('execute')) {
        this.execute = initObj.execute
      }
      else {
        this.execute = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveJointRequest
    // Serialize message field [val]
    bufferOffset = _arraySerializer.float64(obj.val, buffer, bufferOffset, null);
    // Serialize message field [execute]
    bufferOffset = _serializer.bool(obj.execute, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveJointRequest
    let len;
    let data = new MoveJointRequest(null);
    // Deserialize message field [val]
    data.val = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [execute]
    data.execute = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.val.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MoveJointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9c07c95fac99ad0dd277ffa3486c1adc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] val
    bool execute
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveJointRequest(null);
    if (msg.val !== undefined) {
      resolved.val = msg.val;
    }
    else {
      resolved.val = []
    }

    if (msg.execute !== undefined) {
      resolved.execute = msg.execute;
    }
    else {
      resolved.execute = false
    }

    return resolved;
    }
};

class MoveJointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveJointResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveJointResponse
    let len;
    let data = new MoveJointResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MoveJointResponse';
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
    const resolved = new MoveJointResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: MoveJointRequest,
  Response: MoveJointResponse,
  md5sum() { return '9c07c95fac99ad0dd277ffa3486c1adc'; },
  datatype() { return 'moveit_planner/MoveJoint'; }
};
