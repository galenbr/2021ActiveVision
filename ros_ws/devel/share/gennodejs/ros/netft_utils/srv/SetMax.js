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

class SetMaxRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.forceMax = null;
      this.torqueMax = null;
    }
    else {
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
    // Serializes a message object of type SetMaxRequest
    // Serialize message field [forceMax]
    bufferOffset = _serializer.float64(obj.forceMax, buffer, bufferOffset);
    // Serialize message field [torqueMax]
    bufferOffset = _serializer.float64(obj.torqueMax, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMaxRequest
    let len;
    let data = new SetMaxRequest(null);
    // Deserialize message field [forceMax]
    data.forceMax = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torqueMax]
    data.torqueMax = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/SetMaxRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83b478659c66695e91b11173ad5854f9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 forceMax
    float64 torqueMax
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetMaxRequest(null);
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

class SetMaxResponse {
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
    // Serializes a message object of type SetMaxResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetMaxResponse
    let len;
    let data = new SetMaxResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/SetMaxResponse';
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
    const resolved = new SetMaxResponse(null);
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
  Request: SetMaxRequest,
  Response: SetMaxResponse,
  md5sum() { return '10e2f2a95c7447ec4aaa55f68d560d78'; },
  datatype() { return 'netft_utils/SetMax'; }
};
