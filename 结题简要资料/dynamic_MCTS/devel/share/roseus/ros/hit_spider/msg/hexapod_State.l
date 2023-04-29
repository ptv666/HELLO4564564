;; Auto-generated. Do not edit!


(when (boundp 'hit_spider::hexapod_State)
  (if (not (find-package "HIT_SPIDER"))
    (make-package "HIT_SPIDER"))
  (shadow 'hexapod_State (find-package "HIT_SPIDER")))
(unless (find-package "HIT_SPIDER::HEXAPOD_STATE")
  (make-package "HIT_SPIDER::HEXAPOD_STATE"))

(in-package "ROS")
;;//! \htmlinclude hexapod_State.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hit_spider::hexapod_State
  :super ros::object
  :slots (_base_Pose_Now _support_State_Now _faultLeg_State_Now _feetPositionNow _base_Pose_Next _support_State_Next _faultLeg_State_Next _feetPositionNext _move_Direction _remarks ))

(defmethod hit_spider::hexapod_State
  (:init
   (&key
    ((:base_Pose_Now __base_Pose_Now) (instance hit_spider::hexapod_Base_Pose :init))
    ((:support_State_Now __support_State_Now) (make-array 6 :initial-element 0 :element-type :integer))
    ((:faultLeg_State_Now __faultLeg_State_Now) (make-array 6 :initial-element 0 :element-type :integer))
    ((:feetPositionNow __feetPositionNow) (instance hit_spider::FeetPosition :init))
    ((:base_Pose_Next __base_Pose_Next) (instance hit_spider::hexapod_Base_Pose :init))
    ((:support_State_Next __support_State_Next) (make-array 6 :initial-element 0 :element-type :integer))
    ((:faultLeg_State_Next __faultLeg_State_Next) (make-array 6 :initial-element 0 :element-type :integer))
    ((:feetPositionNext __feetPositionNext) (instance hit_spider::FeetPosition :init))
    ((:move_Direction __move_Direction) (instance geometry_msgs::Point :init))
    ((:remarks __remarks) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _base_Pose_Now __base_Pose_Now)
   (setq _support_State_Now __support_State_Now)
   (setq _faultLeg_State_Now __faultLeg_State_Now)
   (setq _feetPositionNow __feetPositionNow)
   (setq _base_Pose_Next __base_Pose_Next)
   (setq _support_State_Next __support_State_Next)
   (setq _faultLeg_State_Next __faultLeg_State_Next)
   (setq _feetPositionNext __feetPositionNext)
   (setq _move_Direction __move_Direction)
   (setq _remarks __remarks)
   self)
  (:base_Pose_Now
   (&rest __base_Pose_Now)
   (if (keywordp (car __base_Pose_Now))
       (send* _base_Pose_Now __base_Pose_Now)
     (progn
       (if __base_Pose_Now (setq _base_Pose_Now (car __base_Pose_Now)))
       _base_Pose_Now)))
  (:support_State_Now
   (&optional __support_State_Now)
   (if __support_State_Now (setq _support_State_Now __support_State_Now)) _support_State_Now)
  (:faultLeg_State_Now
   (&optional __faultLeg_State_Now)
   (if __faultLeg_State_Now (setq _faultLeg_State_Now __faultLeg_State_Now)) _faultLeg_State_Now)
  (:feetPositionNow
   (&rest __feetPositionNow)
   (if (keywordp (car __feetPositionNow))
       (send* _feetPositionNow __feetPositionNow)
     (progn
       (if __feetPositionNow (setq _feetPositionNow (car __feetPositionNow)))
       _feetPositionNow)))
  (:base_Pose_Next
   (&rest __base_Pose_Next)
   (if (keywordp (car __base_Pose_Next))
       (send* _base_Pose_Next __base_Pose_Next)
     (progn
       (if __base_Pose_Next (setq _base_Pose_Next (car __base_Pose_Next)))
       _base_Pose_Next)))
  (:support_State_Next
   (&optional __support_State_Next)
   (if __support_State_Next (setq _support_State_Next __support_State_Next)) _support_State_Next)
  (:faultLeg_State_Next
   (&optional __faultLeg_State_Next)
   (if __faultLeg_State_Next (setq _faultLeg_State_Next __faultLeg_State_Next)) _faultLeg_State_Next)
  (:feetPositionNext
   (&rest __feetPositionNext)
   (if (keywordp (car __feetPositionNext))
       (send* _feetPositionNext __feetPositionNext)
     (progn
       (if __feetPositionNext (setq _feetPositionNext (car __feetPositionNext)))
       _feetPositionNext)))
  (:move_Direction
   (&rest __move_Direction)
   (if (keywordp (car __move_Direction))
       (send* _move_Direction __move_Direction)
     (progn
       (if __move_Direction (setq _move_Direction (car __move_Direction)))
       _move_Direction)))
  (:remarks
   (&rest __remarks)
   (if (keywordp (car __remarks))
       (send* _remarks __remarks)
     (progn
       (if __remarks (setq _remarks (car __remarks)))
       _remarks)))
  (:serialization-length
   ()
   (+
    ;; hit_spider/hexapod_Base_Pose _base_Pose_Now
    (send _base_Pose_Now :serialization-length)
    ;; int8[6] _support_State_Now
    (* 1    6)
    ;; int8[6] _faultLeg_State_Now
    (* 1    6)
    ;; hit_spider/FeetPosition _feetPositionNow
    (send _feetPositionNow :serialization-length)
    ;; hit_spider/hexapod_Base_Pose _base_Pose_Next
    (send _base_Pose_Next :serialization-length)
    ;; int8[6] _support_State_Next
    (* 1    6)
    ;; int8[6] _faultLeg_State_Next
    (* 1    6)
    ;; hit_spider/FeetPosition _feetPositionNext
    (send _feetPositionNext :serialization-length)
    ;; geometry_msgs/Point _move_Direction
    (send _move_Direction :serialization-length)
    ;; std_msgs/String _remarks
    (send _remarks :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; hit_spider/hexapod_Base_Pose _base_Pose_Now
       (send _base_Pose_Now :serialize s)
     ;; int8[6] _support_State_Now
     (dotimes (i 6)
       (write-byte (elt _support_State_Now i) s)
       )
     ;; int8[6] _faultLeg_State_Now
     (dotimes (i 6)
       (write-byte (elt _faultLeg_State_Now i) s)
       )
     ;; hit_spider/FeetPosition _feetPositionNow
       (send _feetPositionNow :serialize s)
     ;; hit_spider/hexapod_Base_Pose _base_Pose_Next
       (send _base_Pose_Next :serialize s)
     ;; int8[6] _support_State_Next
     (dotimes (i 6)
       (write-byte (elt _support_State_Next i) s)
       )
     ;; int8[6] _faultLeg_State_Next
     (dotimes (i 6)
       (write-byte (elt _faultLeg_State_Next i) s)
       )
     ;; hit_spider/FeetPosition _feetPositionNext
       (send _feetPositionNext :serialize s)
     ;; geometry_msgs/Point _move_Direction
       (send _move_Direction :serialize s)
     ;; std_msgs/String _remarks
       (send _remarks :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; hit_spider/hexapod_Base_Pose _base_Pose_Now
     (send _base_Pose_Now :deserialize buf ptr-) (incf ptr- (send _base_Pose_Now :serialization-length))
   ;; int8[6] _support_State_Now
   (dotimes (i (length _support_State_Now))
     (setf (elt _support_State_Now i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _support_State_Now i) 127) (setf (elt _support_State_Now i) (- (elt _support_State_Now i) 256)))
     )
   ;; int8[6] _faultLeg_State_Now
   (dotimes (i (length _faultLeg_State_Now))
     (setf (elt _faultLeg_State_Now i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _faultLeg_State_Now i) 127) (setf (elt _faultLeg_State_Now i) (- (elt _faultLeg_State_Now i) 256)))
     )
   ;; hit_spider/FeetPosition _feetPositionNow
     (send _feetPositionNow :deserialize buf ptr-) (incf ptr- (send _feetPositionNow :serialization-length))
   ;; hit_spider/hexapod_Base_Pose _base_Pose_Next
     (send _base_Pose_Next :deserialize buf ptr-) (incf ptr- (send _base_Pose_Next :serialization-length))
   ;; int8[6] _support_State_Next
   (dotimes (i (length _support_State_Next))
     (setf (elt _support_State_Next i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _support_State_Next i) 127) (setf (elt _support_State_Next i) (- (elt _support_State_Next i) 256)))
     )
   ;; int8[6] _faultLeg_State_Next
   (dotimes (i (length _faultLeg_State_Next))
     (setf (elt _faultLeg_State_Next i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _faultLeg_State_Next i) 127) (setf (elt _faultLeg_State_Next i) (- (elt _faultLeg_State_Next i) 256)))
     )
   ;; hit_spider/FeetPosition _feetPositionNext
     (send _feetPositionNext :deserialize buf ptr-) (incf ptr- (send _feetPositionNext :serialization-length))
   ;; geometry_msgs/Point _move_Direction
     (send _move_Direction :deserialize buf ptr-) (incf ptr- (send _move_Direction :serialization-length))
   ;; std_msgs/String _remarks
     (send _remarks :deserialize buf ptr-) (incf ptr- (send _remarks :serialization-length))
   ;;
   self)
  )

(setf (get hit_spider::hexapod_State :md5sum-) "c1c5c5e8f9a08bf09da5670b143ed75d")
(setf (get hit_spider::hexapod_State :datatype-) "hit_spider/hexapod_State")
(setf (get hit_spider::hexapod_State :definition-)
      "#当前机器人状态
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

")



(provide :hit_spider/hexapod_State "c1c5c5e8f9a08bf09da5670b143ed75d")


