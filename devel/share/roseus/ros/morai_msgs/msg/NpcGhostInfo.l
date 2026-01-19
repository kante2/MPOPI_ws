;; Auto-generated. Do not edit!


(when (boundp 'morai_msgs::NpcGhostInfo)
  (if (not (find-package "MORAI_MSGS"))
    (make-package "MORAI_MSGS"))
  (shadow 'NpcGhostInfo (find-package "MORAI_MSGS")))
(unless (find-package "MORAI_MSGS::NPCGHOSTINFO")
  (make-package "MORAI_MSGS::NPCGHOSTINFO"))

(in-package "ROS")
;;//! \htmlinclude NpcGhostInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass morai_msgs::NpcGhostInfo
  :super ros::object
  :slots (_unique_id _name _position _rpy _steering_angle _vehicle_speed _turn_signal _brake_light ))

(defmethod morai_msgs::NpcGhostInfo
  (:init
   (&key
    ((:unique_id __unique_id) 0)
    ((:name __name) "")
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:rpy __rpy) (instance geometry_msgs::Vector3 :init))
    ((:steering_angle __steering_angle) 0.0)
    ((:vehicle_speed __vehicle_speed) 0.0)
    ((:turn_signal __turn_signal) 0)
    ((:brake_light __brake_light) nil)
    )
   (send-super :init)
   (setq _unique_id (round __unique_id))
   (setq _name (string __name))
   (setq _position __position)
   (setq _rpy __rpy)
   (setq _steering_angle (float __steering_angle))
   (setq _vehicle_speed (float __vehicle_speed))
   (setq _turn_signal (round __turn_signal))
   (setq _brake_light __brake_light)
   self)
  (:unique_id
   (&optional __unique_id)
   (if __unique_id (setq _unique_id __unique_id)) _unique_id)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:rpy
   (&rest __rpy)
   (if (keywordp (car __rpy))
       (send* _rpy __rpy)
     (progn
       (if __rpy (setq _rpy (car __rpy)))
       _rpy)))
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:vehicle_speed
   (&optional __vehicle_speed)
   (if __vehicle_speed (setq _vehicle_speed __vehicle_speed)) _vehicle_speed)
  (:turn_signal
   (&optional __turn_signal)
   (if __turn_signal (setq _turn_signal __turn_signal)) _turn_signal)
  (:brake_light
   (&optional (__brake_light :null))
   (if (not (eq __brake_light :null)) (setq _brake_light __brake_light)) _brake_light)
  (:serialization-length
   ()
   (+
    ;; int32 _unique_id
    4
    ;; string _name
    4 (length _name)
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _rpy
    (send _rpy :serialization-length)
    ;; float32 _steering_angle
    4
    ;; float32 _vehicle_speed
    4
    ;; uint8 _turn_signal
    1
    ;; bool _brake_light
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _unique_id
       (write-long _unique_id s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _rpy
       (send _rpy :serialize s)
     ;; float32 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vehicle_speed
       (sys::poke _vehicle_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _turn_signal
       (write-byte _turn_signal s)
     ;; bool _brake_light
       (if _brake_light (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _unique_id
     (setq _unique_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _rpy
     (send _rpy :deserialize buf ptr-) (incf ptr- (send _rpy :serialization-length))
   ;; float32 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vehicle_speed
     (setq _vehicle_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _turn_signal
     (setq _turn_signal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _brake_light
     (setq _brake_light (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get morai_msgs::NpcGhostInfo :md5sum-) "5d6fbe2aa28a8ec30f515b3c0325abac")
(setf (get morai_msgs::NpcGhostInfo :datatype-) "morai_msgs/NpcGhostInfo")
(setf (get morai_msgs::NpcGhostInfo :definition-)
      "int32 unique_id
string name

geometry_msgs/Vector3 position
geometry_msgs/Vector3 rpy

float32 steering_angle      # 조향 각도 ( degree )
float32 vehicle_speed       # 차량 속력 (km/h)
uint8 turn_signal           # 방향지시등 0: off, 1: left, 2: right, 3: hazard
bool brake_light            # 브레이크등 (켜짐/꺼짐)

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :morai_msgs/NpcGhostInfo "5d6fbe2aa28a8ec30f515b3c0325abac")


