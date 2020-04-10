// Auto-generated. Do not edit!

// (in-package netft_utils.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetBiasRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.toBias = null;
      this.forceMax = null;
      this.torqueMax = null;
    }
    else {
      if (initObj.hasOwnProperty('toBias')) {
        this.toBias = initObj.toBias
      }
      else {
        this.toBias = false;
      }
      if (initObj.hasOwnProperty('forceMax')) {
        this.forceMax = initObj.forceMax
      }
      else {
        this.forceMax = 0.0;
      }
      if (initObj.hasOwnProperty('torqueMax')) {
        this.torqueMax = initObj.torqueMax
      }
      else {
        this.torqueMax = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetBiasRequest
    // Serialize message field [toBias]
    bufferOffset = _serializer.bool(obj.toBias, buffer, bufferOffset);
    // Serialize message field [forceMax]
    bufferOffset = _serializer.float64(obj.forceMax, buffer, bufferOffset);
    // Serialize message field [torqueMax]
    bufferOffset = _serializer.float64(obj.torqueMax, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBiasRequest
    let len;
    let data = new SetBiasRequest(null);
    // Deserialize message field [toBias]
    data.toBias = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [forceMax]
    data.forceMax = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torqueMax]
    data.torqueMax = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/SetBiasRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b52f3420f4c01edd09841826073309b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool toBias
    float64 forceMax
    float64 torqueMax
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBiasRequest(null);
    if (msg.toBias !== undefined) {
      resolved.toBias = msg.toBias;
    }
    else {
      resolved.toBias = false
    }

    if (msg.forceMax !== undefined) {
      resolved.forceMax = msg.forceMax;
    }
    else {
      resolved.forceMax = 0.0
    }

    if (msg.torqueMax !== undefined) {
      resolved.torqueMax = msg.torqueMax;
    }
    else {
      resolved.torqueMax = 0.0
    }

    return resolved;
    }
};

class SetBiasResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetBiasResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBiasResponse
    let len;
    let data = new SetBiasResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/SetBiasResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBiasResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetBiasRequest,
  Response: SetBiasResponse,
  md5sum() { return '24c559ab4eda1111c9526db57ba60c9c'; },
  datatype() { return 'netft_utils/SetBias'; }
};
