#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://kxreus/euslisp/kxr-interface.l")
(load "package://excelsus_description/urdf/excelsus.l")



(defclass excelsus-interface
  :super kxr-interface
  :slots ())

(defmethod excelsus-interface
  (:init (robot &rest args &key (namespace nil)
                (type :default-controller)
                (controller-name "fullbody_controller") &allow-other-keys)
   (send-super* :init robot :type type
                :namespace namespace
                :controller-name controller-name
                :input-joint-names (list "FrontRightJoint0"
                                         "FrontRightJoint1"
                                         "FrontRightJoint2"
                                         "FrontRightJoint3"
                                         "FrontRightJoint4"
                                         "FrontLeftJoint0"
                                         "FrontLeftJoint1"
                                         "FrontLeftJoint2"
                                         "FrontLeftJoint3"
                                         "FrontLeftJoint4"
                                         "RearRightJoint0"
                                         "RearRightJoint1"
                                         "RearRightJoint2"
                                         "RearRightJoint3"
                                         "RearRightJoint4"
                                         "RearLeftJoint0"
                                         "RearLeftJoint1"
                                         "RearLeftJoint2"
                                         "RearLeftJoint3"
                                         "RearLeftJoint4")
                args)
   (ros::ros-info "Initializing excelsus-interface")
   )

  )


(defun excelsus-init (&key
                   (namespace nil)
                   (without-ri nil)
                   (create-viewer nil)
                   (controller-name "fullbody_controller"))
  (unless (boundp '*excelsus*)
    (setq *excelsus* (excelsus)))
  (unless without-ri
    (unless (ros::ok) (ros::roseus "excelsus_eus_interface"))
    (unless (boundp '*ri*)
      (setq *ri* (instance excelsus-interface :init *excelsus* :namespace namespace
                           :controller-name controller-name)))

    (ros::spin-once)
    (send *ri* :spin-once)
    (send *excelsus* :angle-vector (send *ri* :state :potentio-vector)))
  (when create-viewer (objects (list *excelsus*))))

(defun servo-off-sequence ()
  (send *ri* :angle-vector (send *excelsus* :init-pose) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :servo-off))
