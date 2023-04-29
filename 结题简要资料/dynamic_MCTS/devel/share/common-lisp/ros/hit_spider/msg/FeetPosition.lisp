; Auto-generated. Do not edit!


(cl:in-package hit_spider-msg)


;//! \htmlinclude FeetPosition.msg.html

(cl:defclass <FeetPosition> (roslisp-msg-protocol:ros-message)
  ((foot
    :reader foot
    :initarg :foot
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 6 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass FeetPosition (<FeetPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FeetPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FeetPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hit_spider-msg:<FeetPosition> is deprecated: use hit_spider-msg:FeetPosition instead.")))

(cl:ensure-generic-function 'foot-val :lambda-list '(m))
(cl:defmethod foot-val ((m <FeetPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hit_spider-msg:foot-val is deprecated.  Use hit_spider-msg:foot instead.")
  (foot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FeetPosition>) ostream)
  "Serializes a message object of type '<FeetPosition>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'foot))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FeetPosition>) istream)
  "Deserializes a message object of type '<FeetPosition>"
  (cl:setf (cl:slot-value msg 'foot) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'foot)))
    (cl:dotimes (i 6)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FeetPosition>)))
  "Returns string type for a message object of type '<FeetPosition>"
  "hit_spider/FeetPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FeetPosition)))
  "Returns string type for a message object of type 'FeetPosition"
  "hit_spider/FeetPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FeetPosition>)))
  "Returns md5sum for a message object of type '<FeetPosition>"
  "4048f7be9c3b6e5d7ee12bb6c2d1d8e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FeetPosition)))
  "Returns md5sum for a message object of type 'FeetPosition"
  "4048f7be9c3b6e5d7ee12bb6c2d1d8e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FeetPosition>)))
  "Returns full string definition for message of type '<FeetPosition>"
  (cl:format cl:nil "geometry_msgs/Point[6] foot~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FeetPosition)))
  "Returns full string definition for message of type 'FeetPosition"
  (cl:format cl:nil "geometry_msgs/Point[6] foot~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FeetPosition>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'foot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FeetPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'FeetPosition
    (cl:cons ':foot (foot msg))
))
