// Auto-generated. Do not edit!

// (in-package pose_estimator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GazeboPoseEstimation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.model_names = null;
      this.model_poses = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('model_names')) {
        this.model_names = initObj.model_names
      }
      else {
        this.model_names = [];
      }
      if (initObj.hasOwnProperty('model_poses')) {
        this.model_poses = initObj.model_poses
      }
      else {
        this.model_poses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GazeboPoseEstimation
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [model_names]
    bufferOffset = _arraySerializer.string(obj.model_names, buffer, bufferOffset, null);
    // Serialize message field [model_poses]
    // Serialize the length for message field [model_poses]
    bufferOffset = _serializer.uint32(obj.model_poses.length, buffer, bufferOffset);
    obj.model_poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GazeboPoseEstimation
    let len;
    let data = new GazeboPoseEstimation(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [model_names]
    data.model_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [model_poses]
    // Deserialize array length for message field [model_poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.model_poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.model_poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.model_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 56 * object.model_poses.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pose_estimator/GazeboPoseEstimation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bbf8867409277371ff3ab48c9274b56';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    string[] model_names
    geometry_msgs/Pose[] model_poses
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
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
    const resolved = new GazeboPoseEstimation(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.model_names !== undefined) {
      resolved.model_names = msg.model_names;
    }
    else {
      resolved.model_names = []
    }

    if (msg.model_poses !== undefined) {
      resolved.model_poses = new Array(msg.model_poses.length);
      for (let i = 0; i < resolved.model_poses.length; ++i) {
        resolved.model_poses[i] = geometry_msgs.msg.Pose.Resolve(msg.model_poses[i]);
      }
    }
    else {
      resolved.model_poses = []
    }

    return resolved;
    }
};

module.exports = GazeboPoseEstimation;
