// Auto-generated. Do not edit!

// (in-package hit_spider.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class FeetPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.foot = null;
    }
    else {
      if (initObj.hasOwnProperty('foot')) {
        this.foot = initObj.foot
      }
      else {
        this.foot = new Array(6).fill(new geometry_msgs.msg.Point());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FeetPosition
    // Check that the constant length array field [foot] has the right length
    if (obj.foot.length !== 6) {
      throw new Error('Unable to serialize array field foot - length must be 6')
    }
    // Serialize message field [foot]
    obj.foot.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FeetPosition
    let len;
    let data = new FeetPosition(null);
    // Deserialize message field [foot]
    len = 6;
    data.foot = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.foot[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hit_spider/FeetPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4048f7be9c3b6e5d7ee12bb6c2d1d8e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[6] foot
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FeetPosition(null);
    if (msg.foot !== undefined) {
      resolved.foot = new Array(6)
      for (let i = 0; i < resolved.foot.length; ++i) {
        if (msg.foot.length > i) {
          resolved.foot[i] = geometry_msgs.msg.Point.Resolve(msg.foot[i]);
        }
        else {
          resolved.foot[i] = new geometry_msgs.msg.Point();
        }
      }
    }
    else {
      resolved.foot = new Array(6).fill(new geometry_msgs.msg.Point())
    }

    return resolved;
    }
};

module.exports = FeetPosition;
