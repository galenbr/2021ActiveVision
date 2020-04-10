// Auto-generated. Do not edit!

// (in-package netft_utils.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Cancel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.toCancel = null;
    }
    else {
      if (initObj.hasOwnProperty('toCancel')) {
        this.toCancel = initObj.toCancel
      }
      else {
        this.toCancel = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Cancel
    // Serialize message field [toCancel]
    bufferOffset = _serializer.bool(obj.toCancel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Cancel
    let len;
    let data = new Cancel(null);
    // Deserialize message field [toCancel]
    data.toCancel = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'netft_utils/Cancel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60c8bbbce9a3c19aed52c3a96b5d87ff';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool toCancel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Cancel(null);
    if (msg.toCancel !== undefined) {
      resolved.toCancel = msg.toCancel;
    }
    else {
      resolved.toCancel = false
    }

    return resolved;
    }
};

module.exports = Cancel;
