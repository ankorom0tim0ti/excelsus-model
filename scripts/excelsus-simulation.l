(load "package://excelsus_description/urdf/excelsus.l")

(ros::roseus "excelsus-control") ;; initialize node for excelsus controller

(ros::rate 100) ;; set ros rate as 100 hz

(ros::roseus-add-msgs "geometry_msgs") ;; import message type geometry_msgs to use cmd_vel

(ros::ros-info "Needed files are loaded")


(defun scalar-multiply-float-vector (vec scalar)
  (let ((result (make-array (length vec) :element-type 'float)))
    (dotimes (i (length vec))
      (setf (aref result i) (* (aref vec i) scalar)))
    result))

(setq *duaration* 20)

(defun stand-up ()
    ;; Lift excelsus legs to stand up
    (send *excelsus* :angle-vector #f(45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 ))
    (send *irtviewer* :draw-objects)
    (send *excelsus* :angle-vector #f(45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0))
    (send *irtviewer* :draw-objects)
    (unix:usleep 200000) ;; This sleep is needed for reflect command to Excelsus
    ; ;; This pose is standard pose.
    (ros::ros-info "TARGET POS")
    (setq *targetposRR* #f(-130 130 -60)) ;; RearRight
    (setq *targetposRL* #f(130 130 -60)) ;; RearLeft
    (setq *targetposFR* #f(-130 -130 -60)) ;; FrontRight
    (setq *targetposFL* #f(130 -130 -60)) ;; FrontLeft
    (ros::ros-info "CURRENT POS")
    (setq *currentposRR* (send *excelsus* :rear-right-leg :end-coords :worldpos))
    (setq *currentposRL* (send *excelsus* :rear-left-leg :end-coords :worldpos))
    (setq *currentposFR* (send *excelsus* :front-right-leg :end-coords :worldpos))
    (setq *currentposFL* (send *excelsus* :front-left-leg :end-coords :worldpos))
    (send *excelsus* :rear-right-leg :inverse-kinematics
        (make-coords :pos *targetposRR*)
        :rotation-axis t
    )
    (send *excelsus* :rear-left-leg :inverse-kinematics
        (make-coords :pos *targetposRL*)
        :rotation-axis t
    )
    (send *excelsus* :front-right-leg :inverse-kinematics
        (make-coords :pos *targetposFR*)
        :rotation-axis t
    )
    (send *excelsus* :front-left-leg :inverse-kinematics
    (make-coords :pos *targetposFL*)
        :rotation-axis t
    )
    (send *irtviewer* :draw-objects)
    (unix:usleep 200000) ;; This sleep is needed for reflect command to Excelsus
)

(defun creep-walk-right ()
    ; set leg position to walk toward right
    (send *excelsus* :front-right-leg :move-end-pos #f(0 0 50))
    (send *excelsus* :FrontRightJoint0 :joint-angle 30)
    (send *excelsus* :front-right-leg :move-end-pos #f(0 0 -50))

    (send *excelsus* :rear-right-leg :move-end-pos #f(0 0 40))
    (send *excelsus* :RearRightJoint0 :joint-angle 30)
    (send *excelsus* :rear-right-leg :move-end-pos #f(0 0 -50))

    (send *excelsus* :front-left-leg :move-end-pos #f(0 0 50))
    (send *excelsus* :FrontLeftJoint0 :joint-angle 30)
    (send *excelsus* :front-left-leg :move-end-pos #f(0 0 -50))

    (send *excelsus* :rear-left-leg :move-end-pos #f(0 0 50))
    (send *excelsus* :RearLeftJoint0 :joint-angle 30)
    (send *excelsus* :rear-left-leg :move-end-pos #f(0 0 -50))


    ; walk toward right
    (send *excelsus* :front-right-leg :move-end-pos #f(-40 0 50))
    (send *excelsus* :front-right-leg :move-end-pos #f(0 0 -50))

    (send *excelsus* :rear-right-leg :move-end-pos #f(-40 0 50))
    (send *excelsus* :rear-right-leg :move-end-pos #f(0 0 -50))

    (send *excelsus* :front-left-leg :move-end-pos #f(-40 0 50))
    (send *excelsus* :front-left-leg :move-end-pos #f(0 0 -50))

    (send *excelsus* :rear-left-leg :move-end-pos #f(-40 0 50))
    (send *excelsus* :rear-left-leg :move-end-pos #f(0 0 -50))

    (send *excelsus* :angle-vector #f(30.0 15.0 30.0 0.0 0.0 
                                    30.0 15.0 30.0 0.0 0.0 
                                    30.0 15.0 30.0 0.0 0.0 
                                    30.0 15.0 30.0 0.0 0.0))
)



;; Function for walking
(defun walk (mixing order move_leg move_center)

    (unless (and (= (length mixing) 8) (= (length order) 4))
        (error "Length of inpute vector was mismatched"))
    (let ((startpoint (copy-seq #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0))) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
        (centerpoint #f(0.0 0.0 0.0))
        (endpoint (v+  #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0) (scalar-multiply-float-vector mixing move_leg))))
        (format t "~%MoveLeg: ~a" endpoint)
        ;; Move each leg
        (dotimes (i 4)
            (let ((leg_index (elt order i)))
                (setf (elt startpoint (* 2 leg_index)) (elt endpoint (* 2 leg_index))) ;; set x
                (setf (elt startpoint (+ (* 2 leg_index) 1)) (elt endpoint (+ (* 2 leg_index) 1))) ;; set y
                (case leg_index
                    (0 (setf  centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos))) 
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 2) 0))
                    (1 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 2) 0))
                    (2 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 2) 0))
                    (3 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (- move_center (* i 0.2))))
                    (setf (elt centerpoint 2) 0))
                    (t (error "Why is this called?"))
                ) ;; set center point and this would need to change --------------------------------------------------------------------
                (let ((front-right-pos #f(0.0 0.0 -60.0)) (front-left-pos #f(0.0 0.0 -60.0)) (rear-right-pos #f(0.0 0.0 -60.0)) (rear-left-pos #f(0.0 0.0 -60.0)))
                    (setf (elt front-right-pos 0) (elt startpoint 0))
                    (setf (elt front-right-pos 1) (elt startpoint 1))
                    (setf (elt front-left-pos 0) (elt startpoint 2))
                    (setf (elt front-left-pos 1) (elt startpoint 3))
                    (setf (elt rear-right-pos 0) (elt startpoint 4))
                    (setf (elt rear-right-pos 1) (elt startpoint 5))
                    (setf (elt rear-left-pos 0) (elt startpoint 6))
                    (setf (elt rear-left-pos 1) (elt startpoint 7))
                    (send *excelsus* :fullbody-inverse-kinematics
                        (list
                            (make-coords :pos front-right-pos)
                            (make-coords :pos front-left-pos)
                            (make-coords :pos rear-right-pos)
                            (make-coords :pos rear-left-pos)
                        )
                        :move-target
                        (list
                            (send *excelsus* :front-right-leg :end-coords)
                            (send *excelsus* :front-left-leg :end-coords)
                            (send *excelsus* :rear-right-leg :end-coords)
                            (send *excelsus* :rear-left-leg :end-coords)
                        )
                        :link-list
                        (list
                            (send *excelsus* :link-list (send *excelsus* :front-right-leg :end-coords :parent))
                            (send *excelsus* :link-list (send *excelsus* :front-left-leg :end-coords :parent))
                            (send *excelsus* :link-list (send *excelsus* :rear-right-leg :end-coords :parent))
                            (send *excelsus* :link-list (send *excelsus* :rear-left-leg :end-coords :parent))
                        )
                        :target-centroid-pos centerpoint
                        :cog-translation-axis nil
                        :root-link-virtual-joint-weight #f(1.0 1.0 1.0 0 0 0)
                    )
                )
                (case leg_index
                    (0 (send *excelsus* :front-right-leg :move-end-pos #f(-5 -5 60)))
                    (1 (send *excelsus* :front-left-leg :move-end-pos #f(5 -5 60)))
                    (2 (send *excelsus* :rear-right-leg :move-end-pos #f(-5 5 60)))
                    (3 (send *excelsus* :rear-left-leg :move-end-pos #f(5 5 60)))
                    (t (error "Why is this called?"))
                )
                (send *irtviewer* :draw-objects)
                (unix:usleep 200000) ;; This sleep is needed for reflect command to Excelsus
                (case leg_index
                    (0 (send *excelsus* :front-right-leg :move-end-pos #f(5 5 -60)))
                    (1 (send *excelsus* :front-left-leg :move-end-pos #f(-5 5 -60)))
                    (2 (send *excelsus* :rear-right-leg :move-end-pos #f(5 -5 -60)))
                    (3 (send *excelsus* :rear-left-leg :move-end-pos #f(-5 -5 -60)))
                    (t (error "Why is this called?"))
                )
                (send *irtviewer* :draw-objects)
                (unix:usleep 200000) ;; This sleep is needed for reflect command to Excelsus
            )
        )
        ; Reset pose
        (send *excelsus* :fullbody-inverse-kinematics
            (list
                (make-coords :pos #f(-130 130 -60))
                (make-coords :pos #f(130 130 -60))
                (make-coords :pos #f(-130 -130 -60))
                (make-coords :pos #f(130 -130 -60))
            )
            :move-target
            (list
                (send *excelsus* :rear-right-leg :end-coords)
                (send *excelsus* :rear-left-leg :end-coords)
                (send *excelsus* :front-right-leg :end-coords)
                (send *excelsus* :front-left-leg :end-coords)
            )
            :link-list
            (list 
                (send *excelsus* :link-list (send *excelsus* :rear-right-leg :end-coords :parent))
                (send *excelsus* :link-list (send *excelsus* :rear-left-leg :end-coords :parent))
                (send *excelsus* :link-list (send *excelsus* :front-right-leg :end-coords :parent))
                (send *excelsus* :link-list (send *excelsus* :front-left-leg :end-coords :parent))
            )
            ; :translation-axis (list t t t t)
            ; :rotation-axis (list t t t t)
            :target-centroid-pos #f(0 0 0)
            :cog-translation-axis :z
            :root-link-virtual-joint-weight #f(1.0 1.0 1.0 0 0 0)
        )
        (send *irtviewer* :draw-objects)
        (unix:usleep 200000) ;; This sleep is needed for reflect command to Excelsus
    nil )
)


(defun elevate (height)
    (let ((front-right-pos #f(-130.0 -130.0 -60.0)) (front-left-pos #f(130.0 -130.0 -60.0)) (rear-right-pos #f(-130.0 130.0 -60.0)) (rear-left-pos #f(130.0 130.0 -60.0)))
        (setf (elt front-right-pos 2) (- -60.0 height))
        (setf (elt front-left-pos 2) (- -60.0 height))
        (setf (elt rear-right-pos 2) (- -60.0 height))
        (setf (elt rear-left-pos 2) (- -60.0 height))
        (ros::ros-info "ELEVATE START")
        (send *excelsus* :fullbody-inverse-kinematics
            (list
                (make-coords :pos front-right-pos)
                (make-coords :pos front-left-pos)
                (make-coords :pos rear-right-pos)
                (make-coords :pos rear-left-pos)
            )
            :move-target
            (list
                (send *excelsus* :front-right-leg :end-coords)
                (send *excelsus* :front-left-leg :end-coords)
                (send *excelsus* :rear-right-leg :end-coords)
                (send *excelsus* :rear-left-leg :end-coords)
            )
            :link-list
            (list
                (send *excelsus* :link-list (send *excelsus* :front-right-leg :end-coords :parent))
                (send *excelsus* :link-list (send *excelsus* :front-left-leg :end-coords :parent))
                (send *excelsus* :link-list (send *excelsus* :rear-right-leg :end-coords :parent))
                (send *excelsus* :link-list (send *excelsus* :rear-left-leg :end-coords :parent))
            )
            :target-centroid-pos #f(0.0 0.0 0.0)
            :cog-translation-axis t
            :root-link-virtual-joint-weight #f(100.0 100.0 100.0 0 0 0)
        )
        (send *irtviewer* :draw-objects)
        (ros::ros-info "ELEVATE STOP")
    )
)

(defun walk-right ()
    (walk #f(-1.0 0.0 -1.0 0.0 -1.0 0.0 -1.0 0.0) (list 0 2 3 1) 36 8) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)
(defun walk-left ()
    (walk #f(1.0 0.0 1.0 0.0 1.0 0.0 1.0 0.0) (list 3 1 0 2) 36 8) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)
(defun walk-forward ()
    (walk #f(0.0 -1.0 0.0 -1.0 0.0 -1.0 0.0 -1.0) (list 1 0 2 3) 30 6) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)
(defun walk-backward ()
    (walk #f(0.0 1.0 0.0 1.0 0.0 1.0 0.0 1.0) (list 2 3 1 0) 30 6) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)

(defun walk-rotate-right ()
    (walk #f(-47.6 82.4 -82.4 -47.6 82.4 47.6 47.6 -82.4) (list 0 2 3 1) 1 0) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)

(defun walk-rotate-left ()
    (walk #f(82.4 -47.6 47.6 82.4 -47.6 -82.4 -82.4 47.6) (list 0 2 3 1) 1 0) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)

(defun walk-right-until-key ()
    (do-until-key
        (walk-right)
        (ros::ros-info "TIME : ~A" (unix:ptimes))
    )
)
(defun walk-left-until-key ()
    (do-until-key
        (walk-left)
        (ros::ros-info "TIME : ~A" (unix:ptimes))
    )
)
(defun walk-forward-until-key ()
    (do-until-key
        (walk-forward)
        (ros::ros-info "TIME : ~A" (unix:ptimes))
    )
)
(defun walk-backward-until-key ()
    (do-until-key
        (walk-backward)
        (ros::ros-info "TIME : ~A" (unix:ptimes))
    )
)
(defun walk-rotate-right-until-key ()
    (do-until-key
        (walk-rotate-right)
        (ros::ros-info "TIME : ~A" (unix:ptimes))
    )
)
(defun walk-rotate-left-until-key ()
    (do-until-key
        (walk-rotate-left)
        (ros::ros-info "TIME : ~A" (unix:ptimes))
    )
)




(defvar *cmd_topic* "/cmd_vel")

(defun bounding-box-array-cb (msg)
    (let
        ((linearx (send msg :linear :x))
        (lineary (send msg :linear :y))
        (linearz (send msg :linear :z))
        (angularx (send msg :angular :x))
        (angulary (send msg :angular :y))
        (angularz (send msg :angular :z)))
        (case linearx
            (1.0 (walk-right))
            (-1.0 (walk-left))
        )
        (case lineary
            (1.0 (walk-forward))
            (-1.0 (walk-backward))
        )
        (case linearz
            (1.0 (elevate 30))
            (-1.0 (elevate -30))
        )
        (case angularz
            (1.0 (walk-rotate-right))
            (-1.0 (walk-rotate-left))
        )
        (case angularx
            (1.0 (stand-up))
        )
        ; (format t "~%Linear : ~a  Angular : ~a" linear angular)
    )
)

(make-irtviewer)
(excelsus)
(objects *excelsus*)
(send *irtviewer* :draw-objects)

(ros::ros-info "Excelsus is now initialized")

(unix:usleep 200000) ;; This sleep is needed for reflect command to Excelsus
(ros::ros-info "Actuator was activated")
(stand-up)
; (ros::ros-info "Excelsus take initial position")
; (unix:sleep 1)
; (ros::ros-info "Excelsus take initial position2")
; (send *excelsus* :rear-right-leg :move-end-pos #f(-10 0 0)) 
; (send *excelsus* :rear-right-leg :move-end-pos #f(20 0 0))
; (send *excelsus* :front-right-leg :move-end-pos #f(-10 0 0)) 
; (send *excelsus* :front-right-leg :move-end-pos #f(20 0 0 ;; this let code to wait until finishing taking pose
; (send *irtviewer* :draw-objects)
; (send *excelsus* :rear-right-leg :move-end-pos #f(20 0 0)) 
; (send *excelsus* :rear-right-leg :move-end-pos #f(-40 0 0))
; (send *excelsus* :front-right-leg :move-end-pos #f(20 0 0)) 
; (send *excelsus* :front-right-leg :move-end-pos #f(-40 0 0 ;; this let code to wait until finishing taking pose
; (send *irtviewer* :draw-objects)
(ros::subscribe *cmd_topic* geometry_msgs::Twist #'bounding-box-array-cb )

(do-until-key
    (ros::spin-once)
    (ros::sleep)
)

