// Auto-generated. Do not edit!

// (in-package assignment1.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SensorFusion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.FusedData = null;
    }
    else {
      if (initObj.hasOwnProperty('FusedData')) {
        this.FusedData = initObj.FusedData
      }
      else {
        this.FusedData = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorFusion
    // Serialize message field [FusedData]
    bufferOffset = _arraySerializer.float32(obj.FusedData, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorFusion
    let len;
    let data = new SensorFusion(null);
    // Deserialize message field [FusedData]
    data.FusedData = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.FusedData.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'assignment1/SensorFusion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '370f547decfe0b4a69841f116fc2c0fe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # SensorFusion.msg
    
    float32[] FusedData
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorFusion(null);
    if (msg.FusedData !== undefined) {
      resolved.FusedData = msg.FusedData;
    }
    else {
      resolved.FusedData = []
    }

    return resolved;
    }
};

module.exports = SensorFusion;
