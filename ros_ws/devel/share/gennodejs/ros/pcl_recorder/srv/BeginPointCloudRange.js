// Auto-generated. Do not edit!

// (in-package pcl_recorder.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class BeginPointCloudRangeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frequency = null;
    }
    else {
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BeginPointCloudRangeRequest
    // Serialize message field [frequency]
    bufferOffset = _serializer.float64(obj.frequency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BeginPointCloudRangeRequest
    let len;
    let data = new BeginPointCloudRangeRequest(null);
    // Deserialize message field [frequency]
    data.frequency = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pcl_recorder/BeginPointCloudRangeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49c627ecd142a5c42775297233551ead';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 frequency
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BeginPointCloudRangeRequest(null);
    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0.0
    }

    return resolved;
    }
};

class BeginPointCloudRangeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.limitSeconds = null;
    }
    else {
      if (initObj.hasOwnProperty('limitSeconds')) {
        this.limitSeconds = initObj.limitSeconds
      }
      else {
        this.limitSeconds = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BeginPointCloudRangeResponse
    // Serialize message field [limitSeconds]
    bufferOffset = _serializer.float64(obj.limitSeconds, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BeginPointCloudRangeResponse
    let len;
    let data = new BeginPointCloudRangeResponse(null);
    // Deserialize message field [limitSeconds]
    data.limitSeconds = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pcl_recorder/BeginPointCloudRangeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9311910f05228bb5cc6df433181f77e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 limitSeconds
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BeginPointCloudRangeResponse(null);
    if (msg.limitSeconds !== undefined) {
      resolved.limitSeconds = msg.limitSeconds;
    }
    else {
      resolved.limitSeconds = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: BeginPointCloudRangeRequest,
  Response: BeginPointCloudRangeResponse,
  md5sum() { return '10432e08567eb968c3444f0fa65097b5'; },
  datatype() { return 'pcl_recorder/BeginPointCloudRange'; }
};
