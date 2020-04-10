// Auto-generated. Do not edit!

// (in-package pose_estimator.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PoseEstimationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PoseEstimationRequest
    // Serialize message field [data]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PoseEstimationRequest
    let len;
    let data = new PoseEstimationRequest(null);
    // Deserialize message field [data]
    data.data = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.data);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pose_estimator/PoseEstimationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5d036aeaa020e7f5e0ddd6aeae0e7d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 data
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
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
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PoseEstimationRequest(null);
    if (msg.data !== undefined) {
      resolved.data = sensor_msgs.msg.PointCloud2.Resolve(msg.data)
    }
    else {
      resolved.data = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

class PoseEstimationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.detected_object_names = null;
      this.detected_object_poses = null;
    }
    else {
      if (initObj.hasOwnProperty('detected_object_names')) {
        this.detected_object_names = initObj.detected_object_names
      }
      else {
        this.detected_object_names = [];
      }
      if (initObj.hasOwnProperty('detected_object_poses')) {
        this.detected_object_poses = initObj.detected_object_poses
      }
      else {
        this.detected_object_poses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PoseEstimationResponse
    // Serialize message field [detected_object_names]
    bufferOffset = _arraySerializer.string(obj.detected_object_names, buffer, bufferOffset, null);
    // Serialize message field [detected_object_poses]
    // Serialize the length for message field [detected_object_poses]
    bufferOffset = _serializer.uint32(obj.detected_object_poses.length, buffer, bufferOffset);
    obj.detected_object_poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PoseEstimationResponse
    let len;
    let data = new PoseEstimationResponse(null);
    // Deserialize message field [detected_object_names]
    data.detected_object_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [detected_object_poses]
    // Deserialize array length for message field [detected_object_poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.detected_object_poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.detected_object_poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.detected_object_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 56 * object.detected_object_poses.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pose_estimator/PoseEstimationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bfa22b7ddcb05be0466df4bd0bd45e82';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] detected_object_names
    geometry_msgs/Pose[] detected_object_poses
    
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
    const resolved = new PoseEstimationResponse(null);
    if (msg.detected_object_names !== undefined) {
      resolved.detected_object_names = msg.detected_object_names;
    }
    else {
      resolved.detected_object_names = []
    }

    if (msg.detected_object_poses !== undefined) {
      resolved.detected_object_poses = new Array(msg.detected_object_poses.length);
      for (let i = 0; i < resolved.detected_object_poses.length; ++i) {
        resolved.detected_object_poses[i] = geometry_msgs.msg.Pose.Resolve(msg.detected_object_poses[i]);
      }
    }
    else {
      resolved.detected_object_poses = []
    }

    return resolved;
    }
};

module.exports = {
  Request: PoseEstimationRequest,
  Response: PoseEstimationResponse,
  md5sum() { return '022a68c5fe014e6ff7c9f68c41199bb3'; },
  datatype() { return 'pose_estimator/PoseEstimation'; }
};
