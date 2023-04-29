
(cl:in-package :asdf)

(defsystem "hit_spider-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FeetPosition" :depends-on ("_package_FeetPosition"))
    (:file "_package_FeetPosition" :depends-on ("_package"))
    (:file "hexapod_Base_Pose" :depends-on ("_package_hexapod_Base_Pose"))
    (:file "_package_hexapod_Base_Pose" :depends-on ("_package"))
    (:file "hexapod_RPY" :depends-on ("_package_hexapod_RPY"))
    (:file "_package_hexapod_RPY" :depends-on ("_package"))
    (:file "hexapod_State" :depends-on ("_package_hexapod_State"))
    (:file "_package_hexapod_State" :depends-on ("_package"))
  ))