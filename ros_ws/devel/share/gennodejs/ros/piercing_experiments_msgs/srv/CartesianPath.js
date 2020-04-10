// Auto-generated. Do not edit!

// (in-package piercing_experiments_msgs.srv)


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

class CartesianPathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.final_target = null;
      this.vel = null;
    }
    else {
      if (initObj.hasOwnProperty('final_target')) {
        this.final_target = initObj.final_target
      }
      else {
        this.final_target = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianPathRequest
    // Serialize message field [final_target]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.final_target, buffer, bufferOffset);
    // Serialize message field [vel]
    bufferOffset = _serializer.float32(obj.vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianPathRequest
    let len;
    let data = new CartesianPathRequest(null);
    // Deserialize message field [final_target]
    data.final_target = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 60;
  }

  static datatype() {
    // Returns string type for a service object
    return 'piercing_experiments_msgs/CartesianPathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c01a67c204f217a13fc2a48450b5290b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose final_target
    float32 vel
    
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
    const resolved = new CartesianPathRequest(null);
    if (msg.final_target !== undefined) {
      resolved.final_target = geometry_msgs.msg.Pose.Resolve(msg.final_target)
    }
    else {
      resolved.final_target = new geometry_msgs.msg.Pose()
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = 0.0
    }

    return resolved;
    }
};

class CartesianPathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianPathResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianPathResponse
    let len;
    let data = new CartesianPathResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'piercing_experiments_msgs/CartesianPathResponse';
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
    const resolved = new CartesianPathResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: CartesianPathRequest,
  Response: CartesianPathResponse,
  md5sum() { return 'c01a67c204f217a13fc2a48450b5290b'; },
  datatype() { return 'piercing_experiments_msgs/CartesianPath'; }
};
