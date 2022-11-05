;; Auto-generated. Do not edit!


(when (boundp 'car_msgs::CarCmd)
  (if (not (find-package "CAR_MSGS"))
    (make-package "CAR_MSGS"))
  (shadow 'CarCmd (find-package "CAR_MSGS")))
(unless (find-package "CAR_MSGS::CARCMD")
  (make-package "CAR_MSGS::CARCMD"))

(in-package "ROS")
;;//! \htmlinclude CarCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass car_msgs::CarCmd
  :super ros::object
  :slots (_header _a _delta ))

(defmethod car_msgs::CarCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:a __a) 0.0)
    ((:delta __delta) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _a (float __a))
   (setq _delta (float __delta))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:delta
   (&optional __delta)
   (if __delta (setq _delta __delta)) _delta)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _a
    8
    ;; float64 _delta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _a
       (sys::poke _a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta
       (sys::poke _delta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _a
     (setq _a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta
     (setq _delta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get car_msgs::CarCmd :md5sum-) "e19a807f13f32a7ea46f4837ebb5296d")
(setf (get car_msgs::CarCmd :datatype-) "car_msgs/CarCmd")
(setf (get car_msgs::CarCmd :definition-)
      "Header header
float64 a
float64 delta
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :car_msgs/CarCmd "e19a807f13f32a7ea46f4837ebb5296d")


