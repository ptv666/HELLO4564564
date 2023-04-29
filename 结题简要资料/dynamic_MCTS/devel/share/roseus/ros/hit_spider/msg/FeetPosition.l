;; Auto-generated. Do not edit!


(when (boundp 'hit_spider::FeetPosition)
  (if (not (find-package "HIT_SPIDER"))
    (make-package "HIT_SPIDER"))
  (shadow 'FeetPosition (find-package "HIT_SPIDER")))
(unless (find-package "HIT_SPIDER::FEETPOSITION")
  (make-package "HIT_SPIDER::FEETPOSITION"))

(in-package "ROS")
;;//! \htmlinclude FeetPosition.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass hit_spider::FeetPosition
  :super ros::object
  :slots (_foot ))

(defmethod hit_spider::FeetPosition
  (:init
   (&key
    ((:foot __foot) (let (r) (dotimes (i 6) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _foot __foot)
   self)
  (:foot
   (&rest __foot)
   (if (keywordp (car __foot))
       (send* _foot __foot)
     (progn
       (if __foot (setq _foot (car __foot)))
       _foot)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[6] _foot
    (apply #'+ (send-all _foot :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[6] _foot
     (dolist (elem _foot)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[6] _foot
   (dotimes (i 6)
     (send (elt _foot i) :deserialize buf ptr-) (incf ptr- (send (elt _foot i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get hit_spider::FeetPosition :md5sum-) "4048f7be9c3b6e5d7ee12bb6c2d1d8e4")
(setf (get hit_spider::FeetPosition :datatype-) "hit_spider/FeetPosition")
(setf (get hit_spider::FeetPosition :definition-)
      "geometry_msgs/Point[6] foot
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :hit_spider/FeetPosition "4048f7be9c3b6e5d7ee12bb6c2d1d8e4")


