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

class SetFilterRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.toFilter = null;
      this.deltaT = null;
      this.cutoffFrequency = null;
    }
    else {
      if (initObj.hasOwnProperty('toFilter')) {
        this.toFilter = initObj.toFilter
      }
      else {
        this.toFilter = false;
      }
      if (initObj.hasOwnProperty('deltaT')) {
        this.deltaT = initObj.deltaT
      }
      else {
        this.deltaT = 0.0;
      }
      if (initObj.hasOwnProperty('cutoffFrequency')) {
        this.cutoffFrequency = initObj.cutoffFrequency
      }
      else {
        this.cutoffFrequency = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFilterRequest
    // Serialize message field [toFilter]
    bufferOffset = _serializer.bool(obj.toFilter, buffer, bufferOffset);
    // Serialize message field [deltaT]
    bufferOffset = _serializer.float64(obj.deltaT, buffer, bufferOffset);
    // Serialize message field [cutoffFrequency]
    bufferOffset = _serializer.float64(obj.cutoffFrequency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFilterRequest
    let len;
    let data = new SetFilterRequest(null);
    // Deserialize message field [toFilter]
    data.toFilter = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [deltaT]
    data.deltaT = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [cutoffFrequency]
    data.cutoffFrequency = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/SetFilterRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '63edfd1498649d874534855980e23bf0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool toFilter
    float64 deltaT
    float64 cutoffFrequency
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetFilterRequest(null);
    if (msg.toFilter !== undefined) {
      resolved.toFilter = msg.toFilter;
    }
    else {
      resolved.toFilter = false
    }

    if (msg.deltaT !== undefined) {
      resolved.deltaT = msg.deltaT;
    }
    else {
      resolved.deltaT = 0.0
    }

    if (msg.cutoffFrequency !== undefined) {
      resolved.cutoffFrequency = msg.cutoffFrequency;
    }
    else {
      resolved.cutoffFrequency = 0.0
    }

    return resolved;
    }
};

class SetFilterResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFilterResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFilterResponse
    let len;
    let data = new SetFilterResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/SetFilterResponse';
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
    const resolved = new SetFilterResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetFilterRequest,
  Response: SetFilterResponse,
  md5sum() { return '63edfd1498649d874534855980e23bf0'; },
  datatype() { return 'netft_utils/SetFilter'; }
};
