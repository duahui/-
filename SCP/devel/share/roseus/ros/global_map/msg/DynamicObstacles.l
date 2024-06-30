;; Auto-generated. Do not edit!


(when (boundp 'global_map::DynamicObstacles)
  (if (not (find-package "GLOBAL_MAP"))
    (make-package "GLOBAL_MAP"))
  (shadow 'DynamicObstacles (find-package "GLOBAL_MAP")))
(unless (find-package "GLOBAL_MAP::DYNAMICOBSTACLES")
  (make-package "GLOBAL_MAP::DYNAMICOBSTACLES"))

(in-package "ROS")
;;//! \htmlinclude DynamicObstacles.msg.html


(defclass global_map::DynamicObstacles
  :super ros::object
  :slots (_obstacles ))

(defmethod global_map::DynamicObstacles
  (:init
   (&key
    ((:obstacles __obstacles) (let (r) (dotimes (i 0) (push (instance global_map::DynamicObstacle :init) r)) r))
    )
   (send-super :init)
   (setq _obstacles __obstacles)
   self)
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; global_map/DynamicObstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; global_map/DynamicObstacle[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; global_map/DynamicObstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance global_map::DynamicObstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get global_map::DynamicObstacles :md5sum-) "30b207dd63b971b7a3a28ba13f187795")
(setf (get global_map::DynamicObstacles :datatype-) "global_map/DynamicObstacles")
(setf (get global_map::DynamicObstacles :definition-)
      "DynamicObstacle[] obstacles

================================================================================
MSG: global_map/DynamicObstacle
geometry_msgs/Polygon polygon
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



(provide :global_map/DynamicObstacles "30b207dd63b971b7a3a28ba13f187795")


