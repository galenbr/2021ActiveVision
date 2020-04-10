// Auto-generated. Do not edit!

// (in-package assist_feeding.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class finger_orientationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finger_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('finger_angle')) {
        this.finger_angle = initObj.finger_angle
      }
      else {
        this.finger_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type finger_orientationRequest
    // Serialize message field [finger_angle]
    bufferOffset = _serializer.float64(obj.finger_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type finger_orientationRequest
    let len;
    let data = new finger_orientationRequest(null);
    // Deserialize message field [finger_angle]
    data.finger_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'assist_feeding/finger_orientationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2649f48d233c8c347b13cd50f5edf8d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 finger_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new finger_orientationRequest(null);
    if (msg.finger_angle !== undefined) {
      resolved.finger_angle = msg.finger_angle;
    }
    else {
      resolved.finger_angle = 0.0
    }

    return resolved;
    }
};

class finger_orientationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type finger_orientationResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type finger_orientationResponse
    let len;
    let data = new finger_orientationResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'assist_feeding/finger_orientationResponse';
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
    const resolved = new finger_orientationResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: finger_orientationRequest,
  Response: finger_orientationResponse,
  md5sum() { return '2649f48d233c8c347b13cd50f5edf8d2'; },
  datatype() { return 'assist_feeding/finger_orientation'; }
};
