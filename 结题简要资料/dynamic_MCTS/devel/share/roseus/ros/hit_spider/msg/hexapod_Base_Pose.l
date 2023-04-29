;; Auto-generated. Do not edit!


(when (boundp 'hit_spider::hexapod_Base_Pose)
  (if (not (find-package "HIT_SPIDER"))
    (make-package "HIT_SPIDER"))
  (shadow 'hexapod_Base_Pose (find-package "HIT_SPIDER")))
(unless (find-package "HIT_SPIDER::HEXAPOD_BASE_POSE")
  (make-package "HIT_SPIDER::HEXAPOD_BASE_POSE"))

(in-package "ROS")
;;//! \htmlinclude hexapod_Base_Pose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass hit_spider::hexapod_Base_Pose
  :super ros::object
  :slots (_position _orientation ))

(defmethod hit_spider::hexapod_Base_Pose
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:orientation __orientation) (instance hit_spider::hexapod_RPY :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _orientation __orientation)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; hit_spider/hexapod_RPY _orientation
    (send _orientation :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; hit_spider/hexapod_RPY _orientation
       (send _orientation :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; hit_spider/hexapod_RPY _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;;
   self)
  )

(setf (get hit_spider::hexapod_Base_Pose :md5sum-) "f1b4a886328450f637af10145259080b")
(setf (get hit_spider::hexapod_Base_Pose :datatype-) "hit_spider/hexapod_Base_Pose")
(setf (get hit_spider::hexapod_Base_Pose :definition-)
      "geometry_msgs/Point position
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
")



(provide :hit_spider/hexapod_Base_Pose "f1b4a886328450f637af10145259080b")


