; Auto-generated. Do not edit!


(cl:in-package hit_spider-msg)


;//! \htmlinclude hexapod_State.msg.html

(cl:defclass <hexapod_State> (roslisp-msg-protocol:ros-message)
  ((base_Pose_Now
    :reader base_Pose_Now
    :initarg :base_Pose_Now
    :type hit_spider-msg:hexapod_Base_Pose
    :initform (cl:make-instance 'hit_spider-msg:hexapod_Base_Pose))
   (support_State_Now
    :reader support_State_Now
    :initarg :support_State_Now
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (faultLeg_State_Now
    :reader faultLeg_State_Now
    :initarg :faultLeg_State_Now
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (feetPositionNow
    :reader feetPositionNow
    :initarg :feetPositionNow
    :type hit_spider-msg:FeetPosition
    :initform (cl:make-instance 'hit_spider-msg:FeetPosition))
   (base_Pose_Next
    :reader base_Pose_Next
    :initarg :base_Pose_Next
    :type hit_spider-msg:hexapod_Base_Pose
    :initform (cl:make-instance 'hit_spider-msg:hexapod_Base_Pose))
   (support_State_Next
    :reader support_State_Next
    :initarg :support_State_Next
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (faultLeg_State_Next
    :reader faultLeg_State_Next
    :initarg :faultLeg_State_Next
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (feetPositionNext
    :reader feetPositionNext
    :initarg :feetPositionNext
    :type hit_spider-msg:FeetPosition
    :initform (cl:make-instance 'hit_spider-msg:FeetPosition))
   (move_Direction
    :reader move_Direction
    :initarg :move_Direction
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (remarks
    :reader remarks
    :initarg :remarks
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass hexapod_State (<hexapod_State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hexapod_State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hexapod_State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hit_spider-msg:<hexapod_State> is deprecated: use hit_spider-msg:hexapod_State instead.")))

(cl:ensure-generic-function 'base_Pose_Now-val :lambda-list '(m))
(cl:defmethod base_Pose_Now-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:base_Pose_Now-val is deprecated.  Use hit_spider-msg:base_Pose_Now instead.")
  (base_Pose_Now m))

(cl:ensure-generic-function 'support_State_Now-val :lambda-list '(m))
(cl:defmethod support_State_Now-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:support_State_Now-val is deprecated.  Use hit_spider-msg:support_State_Now instead.")
  (support_State_Now m))

(cl:ensure-generic-function 'faultLeg_State_Now-val :lambda-list '(m))
(cl:defmethod faultLeg_State_Now-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:faultLeg_State_Now-val is deprecated.  Use hit_spider-msg:faultLeg_State_Now instead.")
  (faultLeg_State_Now m))

(cl:ensure-generic-function 'feetPositionNow-val :lambda-list '(m))
(cl:defmethod feetPositionNow-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:feetPositionNow-val is deprecated.  Use hit_spider-msg:feetPositionNow instead.")
  (feetPositionNow m))

(cl:ensure-generic-function 'base_Pose_Next-val :lambda-list '(m))
(cl:defmethod base_Pose_Next-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:base_Pose_Next-val is deprecated.  Use hit_spider-msg:base_Pose_Next instead.")
  (base_Pose_Next m))

(cl:ensure-generic-function 'support_State_Next-val :lambda-list '(m))
(cl:defmethod support_State_Next-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:support_State_Next-val is deprecated.  Use hit_spider-msg:support_State_Next instead.")
  (support_State_Next m))

(cl:ensure-generic-function 'faultLeg_State_Next-val :lambda-list '(m))
(cl:defmethod faultLeg_State_Next-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:faultLeg_State_Next-val is deprecated.  Use hit_spider-msg:faultLeg_State_Next instead.")
  (faultLeg_State_Next m))

(cl:ensure-generic-function 'feetPositionNext-val :lambda-list '(m))
(cl:defmethod feetPositionNext-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:feetPositionNext-val is deprecated.  Use hit_spider-msg:feetPositionNext instead.")
  (feetPositionNext m))

(cl:ensure-generic-function 'move_Direction-val :lambda-list '(m))
(cl:defmethod move_Direction-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:move_Direction-val is deprecated.  Use hit_spider-msg:move_Direction instead.")
  (move_Direction m))

(cl:ensure-generic-function 'remarks-val :lambda-list '(m))
(cl:defmethod remarks-val ((m <hexapod_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:remarks-val is deprecated.  Use hit_spider-msg:remarks instead.")
  (remarks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hexapod_State>) ostream)
  "Serializes a message object of type '<hexapod_State>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base_Pose_Now) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'support_State_Now))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'faultLeg_State_Now))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feetPositionNow) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base_Pose_Next) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'support_State_Next))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'faultLeg_State_Next))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feetPositionNext) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'move_Direction) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'remarks) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hexapod_State>) istream)
  "Deserializes a message object of type '<hexapod_State>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base_Pose_Now) istream)
  (cl:setf (cl:slot-value msg 'support_State_Now) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'support_State_Now)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
  (cl:setf (cl:slot-value msg 'faultLeg_State_Now) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'faultLeg_State_Now)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feetPositionNow) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base_Pose_Next) istream)
  (cl:setf (cl:slot-value msg 'support_State_Next) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'support_State_Next)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
  (cl:setf (cl:slot-value msg 'faultLeg_State_Next) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'faultLeg_State_Next)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feetPositionNext) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'move_Direction) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'remarks) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hexapod_State>)))
  "Returns string type for a message object of type '<hexapod_State>"
  "hit_spider/hexapod_State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hexapod_State)))
  "Returns string type for a message object of type 'hexapod_State"
  "hit_spider/hexapod_State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hexapod_State>)))
  "Returns md5sum for a message object of type '<hexapod_State>"
  "c1c5c5e8f9a08bf09da5670b143ed75d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hexapod_State)))
  "Returns md5sum for a message object of type 'hexapod_State"
  "c1c5c5e8f9a08bf09da5670b143ed75d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hexapod_State>)))
  "Returns full string definition for message of type '<hexapod_State>"
  (cl:format cl:nil "#当前机器人状态~%hit_spider/hexapod_Base_Pose base_Pose_Now~%int8[6] support_State_Now~%int8[6] faultLeg_State_Now~%hit_spider/FeetPosition feetPositionNow~%~%#下一步机器人状态~%hit_spider/hexapod_Base_Pose base_Pose_Next~%int8[6] support_State_Next~%int8[6] faultLeg_State_Next~%hit_spider/FeetPosition feetPositionNext~%~%#移动方向~%geometry_msgs/Point move_Direction~%~%std_msgs/String remarks~%================================================================================~%MSG: hit_spider/hexapod_Base_Pose~%geometry_msgs/Point position~%hit_spider/hexapod_RPY orientation~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: hit_spider/hexapod_RPY~%float64 roll~%float64 pitch~%float64 yaw~%================================================================================~%MSG: hit_spider/FeetPosition~%geometry_msgs/Point[6] foot~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hexapod_State)))
  "Returns full string definition for message of type 'hexapod_State"
  (cl:format cl:nil "#当前机器人状态~%hit_spider/hexapod_Base_Pose base_Pose_Now~%int8[6] support_State_Now~%int8[6] faultLeg_State_Now~%hit_spider/FeetPosition feetPositionNow~%~%#下一步机器人状态~%hit_spider/hexapod_Base_Pose base_Pose_Next~%int8[6] support_State_Next~%int8[6] faultLeg_State_Next~%hit_spider/FeetPosition feetPositionNext~%~%#移动方向~%geometry_msgs/Point move_Direction~%~%std_msgs/String remarks~%================================================================================~%MSG: hit_spider/hexapod_Base_Pose~%geometry_msgs/Point position~%hit_spider/hexapod_RPY orientation~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: hit_spider/hexapod_RPY~%float64 roll~%float64 pitch~%float64 yaw~%================================================================================~%MSG: hit_spider/FeetPosition~%geometry_msgs/Point[6] foot~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hexapod_State>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base_Pose_Now))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'support_State_Now) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'faultLeg_State_Now) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feetPositionNow))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base_Pose_Next))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'support_State_Next) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'faultLeg_State_Next) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feetPositionNext))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'move_Direction))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'remarks))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hexapod_State>))
  "Converts a ROS message object to a list"
  (cl:list 'hexapod_State
    (cl:cons ':base_Pose_Now (base_Pose_Now msg))
    (cl:cons ':support_State_Now (support_State_Now msg))
    (cl:cons ':faultLeg_State_Now (faultLeg_State_Now msg))
    (cl:cons ':feetPositionNow (feetPositionNow msg))
    (cl:cons ':base_Pose_Next (base_Pose_Next msg))
    (cl:cons ':support_State_Next (support_State_Next msg))
    (cl:cons ':faultLeg_State_Next (faultLeg_State_Next msg))
    (cl:cons ':feetPositionNext (feetPositionNext msg))
    (cl:cons ':move_Direction (move_Direction msg))
    (cl:cons ':remarks (remarks msg))
))
