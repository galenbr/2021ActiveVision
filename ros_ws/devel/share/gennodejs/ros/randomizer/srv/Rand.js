// Auto-generated. Do not edit!

// (in-package randomizer.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class RandRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.toStart = null;
    }
    else {
      if (initObj.hasOwnProperty('toStart')) {
        this.toStart = initObj.toStart
      }
      else {
        this.toStart = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RandRequest
    // Serialize message field [toStart]
    bufferOffset = _serializer.bool(obj.toStart, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RandRequest
    let len;
    let data = new RandRequest(null);
    // Deserialize message field [toStart]
    data.toStart = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'randomizer/RandRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb0da9f8cfc2a14ca80d5a9fa236c5fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool toStart
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RandRequest(null);
    if (msg.toStart !== undefined) {
      resolved.toStart = msg.toStart;
    }
    else {
      resolved.toStart = false
    }

    return resolved;
    }
};

class RandResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RandResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RandResponse
    let len;
    let data = new RandResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'randomizer/RandResponse';
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
    const resolved = new RandResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: RandRequest,
  Response: RandResponse,
  md5sum() { return 'cb0da9f8cfc2a14ca80d5a9fa236c5fd'; },
  datatype() { return 'randomizer/Rand'; }
};
