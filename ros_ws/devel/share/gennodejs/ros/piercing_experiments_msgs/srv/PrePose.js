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

class PrePoseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_1 = null;
    }
    else {
      if (initObj.hasOwnProperty('target_1')) {
        this.target_1 = initObj.target_1
      }
      else {
        this.target_1 = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PrePoseRequest
    // Serialize message field [target_1]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.target_1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PrePoseRequest
    let len;
    let data = new PrePoseRequest(null);
    // Deserialize message field [target_1]
    data.target_1 = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'piercing_experiments_msgs/PrePoseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '315b676814b67387d174953d2c38a6f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose target_1
    
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
    const resolved = new PrePoseRequest(null);
    if (msg.target_1 !== undefined) {
      resolved.target_1 = geometry_msgs.msg.Pose.Resolve(msg.target_1)
    }
    else {
      resolved.target_1 = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

class PrePoseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PrePoseResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PrePoseResponse
    let len;
    let data = new PrePoseResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'piercing_experiments_msgs/PrePoseResponse';
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
    const resolved = new PrePoseResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: PrePoseRequest,
  Response: PrePoseResponse,
  md5sum() { return '315b676814b67387d174953d2c38a6f1'; },
  datatype() { return 'piercing_experiments_msgs/PrePose'; }
};
