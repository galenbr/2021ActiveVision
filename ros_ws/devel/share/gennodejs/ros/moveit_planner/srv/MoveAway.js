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

class MoveAwayRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.distance = null;
      this.execute = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
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
    // Serializes a message object of type MoveAwayRequest
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float64(obj.distance, buffer, bufferOffset);
    // Serialize message field [execute]
    bufferOffset = _serializer.bool(obj.execute, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveAwayRequest
    let len;
    let data = new MoveAwayRequest(null);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [execute]
    data.execute = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 65;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MoveAwayRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1a34fbddbc93cebf67135565018c85d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose pose
    float64 distance
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
    const resolved = new MoveAwayRequest(null);
    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
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

class MoveAwayResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.awayPose = null;
    }
    else {
      if (initObj.hasOwnProperty('awayPose')) {
        this.awayPose = initObj.awayPose
      }
      else {
        this.awayPose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveAwayResponse
    // Serialize message field [awayPose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.awayPose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveAwayResponse
    let len;
    let data = new MoveAwayResponse(null);
    // Deserialize message field [awayPose]
    data.awayPose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_planner/MoveAwayResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc8398ca987797364ed0a2af1ee2f0ea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose awayPose
    
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
    const resolved = new MoveAwayResponse(null);
    if (msg.awayPose !== undefined) {
      resolved.awayPose = geometry_msgs.msg.Pose.Resolve(msg.awayPose)
    }
    else {
      resolved.awayPose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: MoveAwayRequest,
  Response: MoveAwayResponse,
  md5sum() { return 'b0b7b3e5a88a68f810ee5cdc3adb57e7'; },
  datatype() { return 'moveit_planner/MoveAway'; }
};
