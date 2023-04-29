// Auto-generated. Do not edit!

// (in-package hit_spider.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let hexapod_Base_Pose = require('./hexapod_Base_Pose.js');
let FeetPosition = require('./FeetPosition.js');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class hexapod_State {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.base_Pose_Now = null;
      this.support_State_Now = null;
      this.faultLeg_State_Now = null;
      this.feetPositionNow = null;
      this.base_Pose_Next = null;
      this.support_State_Next = null;
      this.faultLeg_State_Next = null;
      this.feetPositionNext = null;
      this.move_Direction = null;
      this.remarks = null;
    }
    else {
      if (initObj.hasOwnProperty('base_Pose_Now')) {
        this.base_Pose_Now = initObj.base_Pose_Now
      }
      else {
        this.base_Pose_Now = new hexapod_Base_Pose();
      }
      if (initObj.hasOwnProperty('support_State_Now')) {
        this.support_State_Now = initObj.support_State_Now
      }
      else {
        this.support_State_Now = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('faultLeg_State_Now')) {
        this.faultLeg_State_Now = initObj.faultLeg_State_Now
      }
      else {
        this.faultLeg_State_Now = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('feetPositionNow')) {
        this.feetPositionNow = initObj.feetPositionNow
      }
      else {
        this.feetPositionNow = new FeetPosition();
      }
      if (initObj.hasOwnProperty('base_Pose_Next')) {
        this.base_Pose_Next = initObj.base_Pose_Next
      }
      else {
        this.base_Pose_Next = new hexapod_Base_Pose();
      }
      if (initObj.hasOwnProperty('support_State_Next')) {
        this.support_State_Next = initObj.support_State_Next
      }
      else {
        this.support_State_Next = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('faultLeg_State_Next')) {
        this.faultLeg_State_Next = initObj.faultLeg_State_Next
      }
      else {
        this.faultLeg_State_Next = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('feetPositionNext')) {
        this.feetPositionNext = initObj.feetPositionNext
      }
      else {
        this.feetPositionNext = new FeetPosition();
      }
      if (initObj.hasOwnProperty('move_Direction')) {
        this.move_Direction = initObj.move_Direction
      }
      else {
        this.move_Direction = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('remarks')) {
        this.remarks = initObj.remarks
      }
      else {
        this.remarks = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hexapod_State
    // Serialize message field [base_Pose_Now]
    bufferOffset = hexapod_Base_Pose.serialize(obj.base_Pose_Now, buffer, bufferOffset);
    // Check that the constant length array field [support_State_Now] has the right length
    if (obj.support_State_Now.length !== 6) {
      throw new Error('Unable to serialize array field support_State_Now - length must be 6')
    }
    // Serialize message field [support_State_Now]
    bufferOffset = _arraySerializer.int8(obj.support_State_Now, buffer, bufferOffset, 6);
    // Check that the constant length array field [faultLeg_State_Now] has the right length
    if (obj.faultLeg_State_Now.length !== 6) {
      throw new Error('Unable to serialize array field faultLeg_State_Now - length must be 6')
    }
    // Serialize message field [faultLeg_State_Now]
    bufferOffset = _arraySerializer.int8(obj.faultLeg_State_Now, buffer, bufferOffset, 6);
    // Serialize message field [feetPositionNow]
    bufferOffset = FeetPosition.serialize(obj.feetPositionNow, buffer, bufferOffset);
    // Serialize message field [base_Pose_Next]
    bufferOffset = hexapod_Base_Pose.serialize(obj.base_Pose_Next, buffer, bufferOffset);
    // Check that the constant length array field [support_State_Next] has the right length
    if (obj.support_State_Next.length !== 6) {
      throw new Error('Unable to serialize array field support_State_Next - length must be 6')
    }
    // Serialize message field [support_State_Next]
    bufferOffset = _arraySerializer.int8(obj.support_State_Next, buffer, bufferOffset, 6);
    // Check that the constant length array field [faultLeg_State_Next] has the right length
    if (obj.faultLeg_State_Next.length !== 6) {
      throw new Error('Unable to serialize array field faultLeg_State_Next - length must be 6')
    }
    // Serialize message field [faultLeg_State_Next]
    bufferOffset = _arraySerializer.int8(obj.faultLeg_State_Next, buffer, bufferOffset, 6);
    // Serialize message field [feetPositionNext]
    bufferOffset = FeetPosition.serialize(obj.feetPositionNext, buffer, bufferOffset);
    // Serialize message field [move_Direction]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.move_Direction, buffer, bufferOffset);
    // Serialize message field [remarks]
    bufferOffset = std_msgs.msg.String.serialize(obj.remarks, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hexapod_State
    let len;
    let data = new hexapod_State(null);
    // Deserialize message field [base_Pose_Now]
    data.base_Pose_Now = hexapod_Base_Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [support_State_Now]
    data.support_State_Now = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [faultLeg_State_Now]
    data.faultLeg_State_Now = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [feetPositionNow]
    data.feetPositionNow = FeetPosition.deserialize(buffer, bufferOffset);
    // Deserialize message field [base_Pose_Next]
    data.base_Pose_Next = hexapod_Base_Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [support_State_Next]
    data.support_State_Next = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [faultLeg_State_Next]
    data.faultLeg_State_Next = _arrayDeserializer.int8(buffer, bufferOffset, 6)
    // Deserialize message field [feetPositionNext]
    data.feetPositionNext = FeetPosition.deserialize(buffer, bufferOffset);
    // Deserialize message field [move_Direction]
    data.move_Direction = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [remarks]
    data.remarks = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.remarks);
    return length + 192;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hit_spider/hexapod_State';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1c5c5e8f9a08bf09da5670b143ed75d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #当前机器人状态
    hit_spider/hexapod_Base_Pose base_Pose_Now
    int8[6] support_State_Now
    int8[6] faultLeg_State_Now
    hit_spider/FeetPosition feetPositionNow
    
    #下一步机器人状态
    hit_spider/hexapod_Base_Pose base_Pose_Next
    int8[6] support_State_Next
    int8[6] faultLeg_State_Next
    hit_spider/FeetPosition feetPositionNext
    
    #移动方向
    geometry_msgs/Point move_Direction
    
    std_msgs/String remarks
    ================================================================================
    MSG: hit_spider/hexapod_Base_Pose
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
    ================================================================================
    MSG: hit_spider/FeetPosition
    geometry_msgs/Point[6] foot
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new hexapod_State(null);
    if (msg.base_Pose_Now !== undefined) {
      resolved.base_Pose_Now = hexapod_Base_Pose.Resolve(msg.base_Pose_Now)
    }
    else {
      resolved.base_Pose_Now = new hexapod_Base_Pose()
    }

    if (msg.support_State_Now !== undefined) {
      resolved.support_State_Now = msg.support_State_Now;
    }
    else {
      resolved.support_State_Now = new Array(6).fill(0)
    }

    if (msg.faultLeg_State_Now !== undefined) {
      resolved.faultLeg_State_Now = msg.faultLeg_State_Now;
    }
    else {
      resolved.faultLeg_State_Now = new Array(6).fill(0)
    }

    if (msg.feetPositionNow !== undefined) {
      resolved.feetPositionNow = FeetPosition.Resolve(msg.feetPositionNow)
    }
    else {
      resolved.feetPositionNow = new FeetPosition()
    }

    if (msg.base_Pose_Next !== undefined) {
      resolved.base_Pose_Next = hexapod_Base_Pose.Resolve(msg.base_Pose_Next)
    }
    else {
      resolved.base_Pose_Next = new hexapod_Base_Pose()
    }

    if (msg.support_State_Next !== undefined) {
      resolved.support_State_Next = msg.support_State_Next;
    }
    else {
      resolved.support_State_Next = new Array(6).fill(0)
    }

    if (msg.faultLeg_State_Next !== undefined) {
      resolved.faultLeg_State_Next = msg.faultLeg_State_Next;
    }
    else {
      resolved.faultLeg_State_Next = new Array(6).fill(0)
    }

    if (msg.feetPositionNext !== undefined) {
      resolved.feetPositionNext = FeetPosition.Resolve(msg.feetPositionNext)
    }
    else {
      resolved.feetPositionNext = new FeetPosition()
    }

    if (msg.move_Direction !== undefined) {
      resolved.move_Direction = geometry_msgs.msg.Point.Resolve(msg.move_Direction)
    }
    else {
      resolved.move_Direction = new geometry_msgs.msg.Point()
    }

    if (msg.remarks !== undefined) {
      resolved.remarks = std_msgs.msg.String.Resolve(msg.remarks)
    }
    else {
      resolved.remarks = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = hexapod_State;
