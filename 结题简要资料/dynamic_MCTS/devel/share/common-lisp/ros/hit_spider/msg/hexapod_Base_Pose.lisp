; Auto-generated. Do not edit!


(cl:in-package hit_spider-msg)


;//! \htmlinclude hexapod_Base_Pose.msg.html

(cl:defclass <hexapod_Base_Pose> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (orientation
    :reader orientation
    :initarg :orientation
    :type hit_spider-msg:hexapod_RPY
    :initform (cl:make-instance 'hit_spider-msg:hexapod_RPY)))
)

(cl:defclass hexapod_Base_Pose (<hexapod_Base_Pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hexapod_Base_Pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hexapod_Base_Pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hit_spider-msg:<hexapod_Base_Pose> is deprecated: use hit_spider-msg:hexapod_Base_Pose instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <hexapod_Base_Pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:position-val is deprecated.  Use hit_spider-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <hexapod_Base_Pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:orientation-val is deprecated.  Use hit_spider-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hexapod_Base_Pose>) ostream)
  "Serializes a message object of type '<hexapod_Base_Pose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hexapod_Base_Pose>) istream)
  "Deserializes a message object of type '<hexapod_Base_Pose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hexapod_Base_Pose>)))
  "Returns string type for a message object of type '<hexapod_Base_Pose>"
  "hit_spider/hexapod_Base_Pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hexapod_Base_Pose)))
  "Returns string type for a message object of type 'hexapod_Base_Pose"
  "hit_spider/hexapod_Base_Pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hexapod_Base_Pose>)))
  "Returns md5sum for a message object of type '<hexapod_Base_Pose>"
  "f1b4a886328450f637af10145259080b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hexapod_Base_Pose)))
  "Returns md5sum for a message object of type 'hexapod_Base_Pose"
  "f1b4a886328450f637af10145259080b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hexapod_Base_Pose>)))
  "Returns full string definition for message of type '<hexapod_Base_Pose>"
  (cl:format cl:nil "geometry_msgs/Point position~%hit_spider/hexapod_RPY orientation~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: hit_spider/hexapod_RPY~%float64 roll~%float64 pitch~%float64 yaw~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hexapod_Base_Pose)))
  "Returns full string definition for message of type 'hexapod_Base_Pose"
  (cl:format cl:nil "geometry_msgs/Point position~%hit_spider/hexapod_RPY orientation~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: hit_spider/hexapod_RPY~%float64 roll~%float64 pitch~%float64 yaw~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hexapod_Base_Pose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hexapod_Base_Pose>))
  "Converts a ROS message object to a list"
  (cl:list 'hexapod_Base_Pose
    (cl:cons ':position (position msg))
    (cl:cons ':orientation (orientation msg))
))
