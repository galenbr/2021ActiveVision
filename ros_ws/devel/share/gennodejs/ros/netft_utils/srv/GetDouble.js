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

class GetDoubleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ignore = null;
    }
    else {
      if (initObj.hasOwnProperty('ignore')) {
        this.ignore = initObj.ignore
      }
      else {
        this.ignore = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDoubleRequest
    // Serialize message field [ignore]
    bufferOffset = _serializer.bool(obj.ignore, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDoubleRequest
    let len;
    let data = new GetDoubleRequest(null);
    // Deserialize message field [ignore]
    data.ignore = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/GetDoubleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24805c685e7661cfbb5dbdde9c235228';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ignore
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDoubleRequest(null);
    if (msg.ignore !== undefined) {
      resolved.ignore = msg.ignore;
    }
    else {
      resolved.ignore = false
    }

    return resolved;
    }
};

class GetDoubleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.weight = null;
    }
    else {
      if (initObj.hasOwnProperty('weight')) {
        this.weight = initObj.weight
      }
      else {
        this.weight = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDoubleResponse
    // Serialize message field [weight]
    bufferOffset = _serializer.float64(obj.weight, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDoubleResponse
    let len;
    let data = new GetDoubleResponse(null);
    // Deserialize message field [weight]
    data.weight = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'netft_utils/GetDoubleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41998f6691705e7d3db1ca4195275ab0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 weight
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDoubleResponse(null);
    if (msg.weight !== undefined) {
      resolved.weight = msg.weight;
    }
    else {
      resolved.weight = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetDoubleRequest,
  Response: GetDoubleResponse,
  md5sum() { return '43623635c0100a1d66762de2a2c1a2da'; },
  datatype() { return 'netft_utils/GetDouble'; }
};
