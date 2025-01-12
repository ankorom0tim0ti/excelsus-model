(load "excelsus.l") ;; load excelsus-interface lisp file

(ros::roseus "excelsus-control") ;; initialize node for excelsus controller

(ros::rate 60) ;; set ros rate as 60 hz

(ros::roseus-add-msgs "geometry_msgs") ;; import message type geometry_msgs to use cmd_vel

(ros::ros-info "Needed files are loaded")

(setq *duaration* 2)
(setq *walk_flag* nil)
(setq *HFR* 0)
(setq *HFL* 0)
(setq *HRR* 0)
(setq *HRL* 0)


(defun scalar-multiply-float-vector (vec scalar)
  (let ((result (make-array (length vec) :element-type 'float)))
    (dotimes (i (length vec))
      (setf (aref result i) (* (aref vec i) scalar)))
    result)
)

(defun on ()
    (send *ri* :servo-on)
)

(defun off ()
    (send *ri* :servo-off)
)

(defun stand-up ()
    (setf *walk_flag* t)
    (ros::ros-info "STAND UP START")
    (send *excelsus* :angle-vector #f(45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 ))
    (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
    (send *ri* :wait-interpolation) ;; this let code to wait until finishing taking pose
    (send *excelsus* :angle-vector #f(45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0))
    (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
    (send *ri* :wait-interpolation) ;; this let code to wait until finishing taking pose

    (setq *targetposRR* #f(-130 130 -60)) ;; RearRight
    (setq *targetposRL* #f(130 130 -60)) ;; RearLeft
    (setq *targetposFR* #f(-130 -130 -60)) ;; FrontRight
    (setq *targetposFL* #f(130 -130 -60)) ;; FrontLeft
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
    (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
    (send *ri* :wait-interpolation) ;; this let code to wait until finishing taking pose
    (ros::ros-info "STAND UP END")
    (setf *walk-flag* nil)
)

;; Function for walking
(defun walk (mixing order move_leg move_center)
    (setf *walk_flag* t)   
    (ros::ros-info "STEP START")
    (elevate 0)
    (unless (and (= (length mixing) 8) (= (length order) 4))
        (error "Length of inpute vector was mismatched"))
    (let ((startpoint (copy-seq #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0))) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
        (centerpoint #f(0.0 0.0 0.0))
        (endpoint (v+  #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0) (scalar-multiply-float-vector mixing move_leg))))
        ;; Move each leg
        (dotimes (i 4)
            (let ((leg_index (elt order i)))
                (setf (elt startpoint (* 2 leg_index)) (elt endpoint (* 2 leg_index))) ;; set x
                (setf (elt startpoint (+ (* 2 leg_index) 1)) (elt endpoint (+ (* 2 leg_index) 1))) ;; set y
                (case leg_index
                    (0 (setf  centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos))) 
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 2) 0))
                    (1 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 2) 0))
                    (2 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 2) 0))
                    (3 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (+ move_center (* 1.2 (* (- i 1.5) (- i 1.5) )))))
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
                    ; (format t "~%MoveLeg: ~a" centerpoint)
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
                        :cog-translation-axis :z
                        :root-link-virtual-joint-weight #f(100.0 100.0 100.0 0 0 0)
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
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)
                (case leg_index
                    (0 (send *excelsus* :front-right-leg :move-end-pos #f(5 5 -60)))
                    (1 (send *excelsus* :front-left-leg :move-end-pos #f(-5 5 -60)))
                    (2 (send *excelsus* :rear-right-leg :move-end-pos #f(5 -5 -60)))
                    (3 (send *excelsus* :rear-left-leg :move-end-pos #f(-5 -5 -60)))
                    (t (error "Why is this called?"))
                )
                (send *irtviewer* :draw-objects)
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)
            )
        )
        ;; Reset pose
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
            :cog-translation-axis t
            :root-link-virtual-joint-weight #f(1.0 1.0 1.0 0 0 0)
            
        )
        ; (send *excelsus* :angle-vector #f(46.9764 2.40906 16.1589 -13.7483 46.9762 47.6174 2.62926 16.2842 -13.6534 47.6172 47.8289 3.1044 16.7837 -13.6777 47.8287 47.1906 2.88624 16.6589 -13.7711 -47.1904)) ;; angle-vector of stand up state
        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
        (send *ri* :wait-interpolation)
        (ros::ros-info "STEP END")
        (setf *walk_flag* nil)
    nil )
)

;; Function for walking
(defun walk-rotate (mixing order move_leg move_center)
    (setf *walk_flag* t)
    (ros::ros-info "STEP START")
    (elevate 0)
    (unless (and (= (length mixing) 8) (= (length order) 4))
        (error "Length of inpute vector was mismatched"))
    (let ((startpoint (copy-seq #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0))) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
        (centerpoint #f(0.0 0.0 0.0))
        (endpoint (v+  #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0) (scalar-multiply-float-vector mixing move_leg))))
        ;; Move each leg
        (dotimes (i 4)
            (let ((leg_index (elt order i)))
                (setf (elt startpoint (* 2 leg_index)) (elt endpoint (* 2 leg_index))) ;; set x
                (setf (elt startpoint (+ (* 2 leg_index) 1)) (elt endpoint (+ (* 2 leg_index) 1))) ;; set y
                (case leg_index
                    (0 (setf  centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos))) 
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 2) 0))
                    (1 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 2) 0))
                    (2 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 2) 0))
                    (3 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1.5) (- i 1.5) )))))
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
                    ; (format t "~%MoveLeg: ~a" centerpoint)
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
                        :root-link-virtual-joint-weight #f(100.0 100.0 100.0 0 0 0)
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
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)
                (case leg_index
                    (0 (send *excelsus* :front-right-leg :move-end-pos #f(5 5 -60)))
                    (1 (send *excelsus* :front-left-leg :move-end-pos #f(-5 5 -60)))
                    (2 (send *excelsus* :rear-right-leg :move-end-pos #f(5 -5 -60)))
                    (3 (send *excelsus* :rear-left-leg :move-end-pos #f(-5 -5 -60)))
                    (t (error "Why is this called?"))
                )
                (send *irtviewer* :draw-objects)
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)
            )
        )
        ;; Reset pose
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
            :cog-translation-axis t
            :root-link-virtual-joint-weight #f(1.0 1.0 1.0 0 0 0)
            
        )
        ; (send *excelsus* :angle-vector #f(46.9764 2.40906 16.1589 -13.7483 46.9762 47.6174 2.62926 16.2842 -13.6534 47.6172 47.8289 3.1044 16.7837 -13.6777 47.8287 47.1906 2.88624 16.6589 -13.7711 -47.1904)) ;; angle-vector of stand up state
        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
        (send *ri* :wait-interpolation)
        (ros::ros-info "STEP END")
        (setf *walk_flag* nil)
    nil )
)

(defun elevate (height)
    (setf *walk_flag* t)
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
        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
        (send *ri* :wait-interpolation)
        (ros::ros-info "ELEVATE STOP")
        (setf *walk_flag* nil)
    )
)



;; Walk toward specified direction
(defun walk-right ()
    (walk #f(-1.0 0.0 -1.0 0.0 -1.0 0.0 -1.0 0.0) (list 0 2 3 1) 32 8) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)
(defun walk-left ()
    (walk #f(1.0 0.0 1.0 0.0 1.0 0.0 1.0 0.0) (list 3 1 0 2) 32 8) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)
(defun walk-forward ()
    (walk #f(0.0 -1.0 0.0 -1.0 0.0 -1.0 0.0 -1.0) (list 1 0 2 3) 30 4) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)
(defun walk-backward ()
    (walk #f(0.0 1.0 0.0 1.0 0.0 1.0 0.0 1.0) (list 2 3 1 0) 30 4) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)

(defun walk-rotate-right ()
    (walk-rotate #f(-47.6 82.4 -82.4 -47.6 82.4 47.6 47.6 -82.4) (list 0 2 3 1) 1 0) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)

(defun walk-rotate-left ()
    (walk-rotate #f(82.4 -47.6 47.6 82.4 -47.6 -82.4 -82.4 47.6) (list 0 2 3 1) 1 0) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
)


;; specified actions
(defun walk-right-until-key ()
    (do-until-key
        (walk-right)
    )
)
(defun walk-left-until-key ()
    (do-until-key
        (walk-left)
    )
)
(defun walk-forward-until-key ()
    (do-until-key
        (walk-forward)
    )
)
(defun walk-backward-until-key ()
    (do-until-key
        (walk-backward)
    )
)
(defun walk-rotate-right-until-key ()
    (do-until-key
        (walk-rotate-right)
    )
)
(defun walk-rotate-left-until-key ()
    (do-until-key
        (walk-rotate-left)
    )
)


(defvar *cmd_topic* "/cmd_vel")

(defvar *stomp_FR0* "/kjs/23/force/0")
(defvar *stomp_FR1* "/kjs/23/force/1")
(defvar *stomp_FR2* "/kjs/23/force/2")
(defvar *stomp_FR3* "/kjs/23/force/3")

(defvar *stomp_FL0* "/kjs/22/force/0")
(defvar *stomp_FL1* "/kjs/22/force/1")
(defvar *stomp_FL2* "/kjs/22/force/2")
(defvar *stomp_FL3* "/kjs/22/force/3")

(defun cmd_cb (msg)
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
            (1.0 (elevate 20))
            (-1.0 (elevate -20))
        )
        (case angularz
            (-1.0 (walk-rotate-right))
            (1.0 (walk-rotate-left))
        )
        (case angularx
            (1.0 (stand-up))
        )
        ; (format t "~%Linear : ~a  Angular : ~a" linear angular)
    )
)

(defun stomp_leg (leg_index)
    (case leg_index
        (0 (let 
                ((msg (ros::topic-get *stomp_FR0* "geometry_msgs/WrenchStamped")))
                (if msg
                    ((let (force (send msg :wrench :force :x))
                        (ros::ros-info "FORCE ~A" force)  
                        )
                    )
                    ;()
                )
            )
        )
        (1)
        (2)
        (3)
    )
)




(excelsus-init) ;; initialize excelsus simulation and robot interface

(ros::ros-info "Excelsus is now initialized")
(send *ri* :servo-on) ;; activate servo motors
(unix:usleep 200000) ;; This sleep is needed for reflect command to Excelsus
(ros::ros-info "Actuator was activated")
(stand-up)
(ros::ros-info "Excelsus take initial position")
(ros::subscribe *cmd_topic* geometry_msgs::Twist #'cmd_cb )
(do-until-key
    (ros::spin-once)
    (ros::sleep)
)
