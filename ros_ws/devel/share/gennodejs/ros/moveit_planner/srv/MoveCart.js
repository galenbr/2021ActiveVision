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

class MoveCartRequest {
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
    // Serializes a message object of type MoveCartRequest
    // Serialize message field [val]
    // Serialize the length for message field [val]
    bufferOffset = _serializer.uint32(obj.val.length, buffer, bufferOffset);
    obj.val.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [execute]
    bufferOffset = _serializer.bool(obj.execute, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveCartRequest
    let len;
    let data = new MoveCartRequest(null);
    // Deserialize message field [val]
    // Deserialize array length for message field [val]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.val = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.val[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [execute]
    data.execute = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 56 * object.val.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MoveCartRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a77c545a10fa137b7b4acf08012288bf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose[] val
    bool execute
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveCartRequest(null);
    if (msg.val !== undefined) {
      resolved.val = new Array(msg.val.length);
      for (let i = 0; i < resolved.val.length; ++i) {
        resolved.val[i] = geometry_msgs.msg.Pose.Resolve(msg.val[i]);
      }
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

class MoveCartResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveCartResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveCartResponse
    let len;
    let data = new MoveCartResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MoveCartResponse';
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
    const resolved = new MoveCartResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: MoveCartRequest,
  Response: MoveCartResponse,
  md5sum() { return 'a77c545a10fa137b7b4acf08012288bf'; },
  datatype() { return 'moveit_planner/MoveCart'; }
};
