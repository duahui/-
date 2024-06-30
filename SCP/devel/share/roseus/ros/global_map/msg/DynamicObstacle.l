;; Auto-generated. Do not edit!


(when (boundp 'global_map::DynamicObstacle)
  (if (not (find-package "GLOBAL_MAP"))
    (make-package "GLOBAL_MAP"))
  (shadow 'DynamicObstacle (find-package "GLOBAL_MAP")))
(unless (find-package "GLOBAL_MAP::DYNAMICOBSTACLE")
  (make-package "GLOBAL_MAP::DYNAMICOBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude DynamicObstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass global_map::DynamicObstacle
  :super ros::object
  :slots (_polygon _trajectory ))

(defmethod global_map::DynamicObstacle
  (:init
   (&key
    ((:polygon __polygon) (instance geometry_msgs::Polygon :init))
    ((:trajectory __trajectory) (let (r) (dotimes (i 0) (push (instance global_map::DynamicTrajectoryPoint :init) r)) r))
    )
   (send-super :init)
   (setq _polygon __polygon)
   (setq _trajectory __trajectory)
   self)
  (:polygon
   (&rest __polygon)
   (if (keywordp (car __polygon))
       (send* _polygon __polygon)
     (progn
       (if __polygon (setq _polygon (car __polygon)))
       _polygon)))
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Polygon _polygon
    (send _polygon :serialization-length)
    ;; global_map/DynamicTrajectoryPoint[] _trajectory
    (apply #'+ (send-all _trajectory :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Polygon _polygon
       (send _polygon :serialize s)
     ;; global_map/DynamicTrajectoryPoint[] _trajectory
     (write-long (length _trajectory) s)
     (dolist (elem _trajectory)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Polygon _polygon
     (send _polygon :deserialize buf ptr-) (incf ptr- (send _polygon :serialization-length))
   ;; global_map/DynamicTrajectoryPoint[] _trajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory (let (r) (dotimes (i n) (push (instance global_map::DynamicTrajectoryPoint :init) r)) r))
     (dolist (elem- _trajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get global_map::DynamicObstacle :md5sum-) "9cbe4735150eb164552f0f587e68e631")
(setf (get global_map::DynamicObstacle :datatype-) "global_map/DynamicObstacle")
(setf (get global_map::DynamicObstacle :definition-)
      "geometry_msgs/Polygon polygon
DynamicTrajectoryPoint[] trajectory

================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
================================================================================
MSG: global_map/DynamicTrajectoryPoint
float64 time
float64 x
float64 y
float64 theta

")



(provide :global_map/DynamicObstacle "9cbe4735150eb164552f0f587e68e631")


