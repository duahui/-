
(cl:in-package :asdf)

(defsystem "map-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "DynamicObstacle" :depends-on ("_package_DynamicObstacle"))
    (:file "_package_DynamicObstacle" :depends-on ("_package"))
    (:file "DynamicObstacles" :depends-on ("_package_DynamicObstacles"))
    (:file "_package_DynamicObstacles" :depends-on ("_package"))
    (:file "DynamicTrajectoryPoint" :depends-on ("_package_DynamicTrajectoryPoint"))
    (:file "_package_DynamicTrajectoryPoint" :depends-on ("_package"))
    (:file "Obstacles" :depends-on ("_package_Obstacles"))
    (:file "_package_Obstacles" :depends-on ("_package"))
  ))