// Auto-generated. Do not edit!

// (in-package moveit_planner.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class MovePointRequest {
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
        this.val = new geometry_msgs.msg.Point();
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
    // Serializes a message object of type MovePointRequest
    // Serialize message field [val]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.val, buffer, bufferOffset);
    // Serialize message field [execute]
    bufferOffset = _serializer.bool(obj.execute, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MovePointRequest
    let len;
    let data = new MovePointRequest(null);
    // Deserialize message field [val]
    data.val = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [execute]
    data.execute = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MovePointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '09d178cfe7c05e5d8c3ba0747602407a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point val
    bool execute
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MovePointRequest(null);
    if (msg.val !== undefined) {
      resolved.val = geometry_msgs.msg.Point.Resolve(msg.val)
    }
    else {
      resolved.val = new geometry_msgs.msg.Point()
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

class MovePointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MovePointResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MovePointResponse
    let len;
    let data = new MovePointResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MovePointResponse';
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
    const resolved = new MovePointResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: MovePointRequest,
  Response: MovePointResponse,
  md5sum() { return '09d178cfe7c05e5d8c3ba0747602407a'; },
  datatype() { return 'moveit_planner/MovePoint'; }
};
