// Auto-generated. Do not edit!

// (in-package hit_spider.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let hexapod_RPY = require('./hexapod_RPY.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class hexapod_Base_Pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.orientation = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new hexapod_RPY();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hexapod_Base_Pose
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = hexapod_RPY.serialize(obj.orientation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hexapod_Base_Pose
    let len;
    let data = new hexapod_Base_Pose(null);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = hexapod_RPY.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hit_spider/hexapod_Base_Pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1b4a886328450f637af10145259080b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point position
    hit_spider/hexapod_RPY orientation
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: hit_spider/hexapod_RPY
    float64 roll
    float64 pitch
    float64 yaw
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new hexapod_Base_Pose(null);
    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = hexapod_RPY.Resolve(msg.orientation)
    }
    else {
      resolved.orientation = new hexapod_RPY()
    }

    return resolved;
    }
};

module.exports = hexapod_Base_Pose;
