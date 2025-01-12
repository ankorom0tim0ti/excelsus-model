;; Load excelsus-interface lisp file
(load "excelsus.l") 

;; Initialize node for excelsus controller
(ros::roseus "excelsus-control")
(ros::ros-info "Intialized Node")

;; Set ros rate as 60 hz
(ros::rate 60)

;; Import message type geometry_msgs to use cmd_vel
(ros::roseus-add-msgs "geometry_msgs")
(ros::ros-info "Needed files are loaded")

;; Values and Parameters ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Set duration of motion
(setq *duaration* 2)

;; Flag for judge whether excelsus is walking or not
(setq *walk_flag* nil)

;; Height of legs
(setq *HFR* -60)
(setq *HFL* -60)
(setq *HRR* -60)
(setq *HRL* -60)

;; Height of raising leg
(setq *LU* 60)

;; Force of leg sensor
(setq *FFR* '(0.0 0.0 0.0 0.0))
(setq *FFL* '(0.0 0.0 0.0 0.0))
(setq *FRR* '(0.0 0.0 0.0 0.0))
(setq *FRL* '(0.0 0.0 0.0 0.0))

;; Reference force of leg sensor
(setq *RFFR* 0)
(setq *RFFL* 0)
(setq *RFRR* 0)
(setq *RFRL* 0)

;; Average force of leg sensor
(setq *MFFR* 0.0)
(setq *MFFL* 0.0)
(setq *MFRR* 0.0)
(setq *MFRL* 0.0)

;; Threshold of force
(setq *THR* 24)


;; Name of ROS topic
(defvar *cmd_topic* "/cmd_vel")

(defvar *stomp_FR0* "/kjs/23/force/0")
(defvar *stomp_FR1* "/kjs/23/force/1")
(defvar *stomp_FR2* "/kjs/23/force/2")
(defvar *stomp_FR3* "/kjs/23/force/3")

(defvar *stomp_FL0* "/kjs/22/force/0")
(defvar *stomp_FL1* "/kjs/22/force/1")
(defvar *stomp_FL2* "/kjs/22/force/2")
(defvar *stomp_FL3* "/kjs/22/force/3")

(defvar *stomp_RR0* "/kjs/24/force/0")
(defvar *stomp_RR1* "/kjs/24/force/1")
(defvar *stomp_RR2* "/kjs/24/force/2")
(defvar *stomp_RR3* "/kjs/24/force/3")

(defvar *stomp_RL0* "/kjs/25/force/0")
(defvar *stomp_RL1* "/kjs/25/force/1")
(defvar *stomp_RL2* "/kjs/25/force/2")
(defvar *stomp_RL3* "/kjs/25/force/3")

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Function to calculate average of float vector
(defun average (list)
    (let (average)
        (setf average 0.0)
        (dolist (i list) 
            (setf average (+ average i))
        );; dolist 0
        (setf average (/ average (length list)))
        average
    );; let 0
);; function

;; Function to multiple float vector with scalar value
(defun scalar-multiply-float-vector (vec scalar)
    (let ((result (make-array (length vec) :element-type 'float)))
        (dotimes (i (length vec))
            (setf (aref result i) (* (aref vec i) scalar))
        );; dotimes 0
        result
    );; let 0
);; function

;; Function for deactivate servo motor
(defun on ()
    (send *ri* :servo-on)
);; function

;; Function for activate servo motor
(defun off ()
    (send *ri* :servo-off)
);; function

;;　Assume standing on flat ground.
(defun stand-up ()
    ;; Turn on walk_flag
    (setf *walk_flag* t)
    (ros::ros-info "STAND UP START")

    ;; pose 1
    (send *excelsus* :angle-vector #f(45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 
                                    45.0 0.0 90.0 -90.0 0.0 ))
    (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
    (send *ri* :wait-interpolation) ;; this let code to wait until finishing taking pose

    ;; pose 2
    (send *excelsus* :angle-vector #f(45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0 
                                    45.0 0.0 30.0 -45.0 0.0))
    (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
    (send *ri* :wait-interpolation) ;; this let code to wait until finishing taking pose

    ;; Calibration Start

    ;; Initialize reference values as 0
    (setf *RFFR* 0)
    (setf *RFFL* 0)
    (setf *RFRR* 0)
    (setf *RFRL* 0)

    ;; Retrive sensor value and integrate them
    (dotimes (i 100)
        (ros::spin-once)
        (setf *RFFR* (+ *RFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3))))
        (setf *RFFL* (+ *RFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3))))
        (setf *RFRR* (+ *RFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3))))
        (setf *RFRL* (+ *RFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3))))
        (ros::sleep )
    );; dotimes 0

    ;; Average the data
    (setf *RFFR* (/ *RFFR* 100))
    (setf *RFFL* (/ *RFFL* 100))
    (setf *RFRR* (/ *RFRR* 100))
    (setf *RFRL* (/ *RFRL* 100))
    (ros::ros-info "reference force: ~A ~A ~A ~A" *RFFR* *RFFL* *RFRR* *RFRL*)

    ;; Take initial position
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

    ;; Initialize height of legs
    (setf *HFR* -60)
    (setf *HFL* -60)
    (setf *HRR* -60)
    (setf *HRL* -60)
    (ros::ros-info "STAND UP END")

    ;; Turn off walk_flag
    (setf *walk_flag* nil)
);; function

;; Function for walking
(defun walk (mixing order move_leg move_center)
    ;; Turn on walk_flag
    (setf *walk_flag* t)   
    (ros::ros-info "WALK START")

    ;; Return the height of the body to the specified value
    (elevate 0)
    (unless (and (= (length mixing) 8) (= (length order) 4))
        (error "Length of inpute vector was mismatched"))
    (let ((startpoint (copy-seq #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0))) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
        (centerpoint #f(0.0 0.0 0.0))
        (endpoint (v+  #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0) (scalar-multiply-float-vector mixing move_leg)))
        (front-right-pos #f(0.0 0.0 -60.0))
        (front-left-pos #f(0.0 0.0 -60.0))
        (rear-right-pos #f(0.0 0.0 -60.0))
        (rear-left-pos #f(0.0 0.0 -60.0))
        (front-right-dif #f(0.0 0.0 0.0))
        (front-left-dif #f(0.0 0.0 0.0))
        (rear-right-dif #f(0.0 0.0 0.0))
        (rear-left-dif #f(0.0 0.0 0.0)))
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

                (case leg_index
                    (0 (setf (elt front-right-dif 2) *LU*) (setf (elt front-left-dif 2) (- *HFL* -60)) (setf (elt rear-right-dif 2) (- *HRR* -60)) (setf (elt rear-left-dif 2) (- *HRR* -60)) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HFR* (- *LU* 60)))
                    (1 (setf (elt front-right-dif 2) (- *HFR* -60)) (setf (elt front-left-dif 2) *LU*) (setf (elt rear-right-dif 2) (- *HRR* -60)) (setf (elt rear-left-dif 2) (- *HRR* -60)) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HFL* (- *LU* 60)))
                    (2 (setf (elt front-right-dif 2) (- *HFR* -60)) (setf (elt front-left-dif 2) (- *HFL* -60)) (setf (elt rear-right-dif 2) *LU*) (setf (elt rear-left-dif 2) (- *HRR* -60)) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HRR* (- *LU* 60)))
                    (3 (setf (elt front-right-dif 2) (- *HFR* -60)) (setf (elt front-left-dif 2) (- *HFL* -60)) (setf (elt rear-right-dif 2) (- *HRR* -60)) (setf (elt rear-left-dif 2) *LU*) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HRL* (- *LU* 60)))
                    (t (error "Why is this called?"))
                )

                ;; (send *irtviewer* :draw-objects)
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)

                (case leg_index
                    (0 (let ((step 1) (flag 0) (PFFR 0) (PFRL 0))
                        (ros::spin-once)
                        (setf PFFR (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                        (setf PFRL (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                        (while (< step 16)
                        (send *excelsus* :front-right-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop front right: ~A ~A" *RFFR* *RFRL*)
                        ;     (if (and (> *MFFR* (+ *RFFR* *THR*)) (> *MFRL* (+ *RFRL* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFFR* (+ PFFR *THR*)) (> *MFRL* (+ *RFRL* *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFFR *MFFR*)
                                    ; (setf PFRL *MFRL*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop front right: ~A ~A ~A ~A" *MFFR* PFFR *MFRL* *RFRL*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "front right stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HFR* (- *HFR* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (1 (let ((step 1) (flag 0) (PFFL 0) (PFRR 0))
                        (ros::spin-once)
                        (setf PFFL (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                        (setf PFRR (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                        (while (< step 16)
                        (send *excelsus* :front-left-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop front left: ~A ~A" *RFFL* *RFRR*)
                        ;     (if (and (> *MFFL* (+ *RFFL* *THR*)) (> *MFRR* (+ *RFRR* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFFL* (+ PFFL *THR*)) (> *MFRR* (+ *RFRR* *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFFL *MFFL*)
                                    ; (setf PFRR *MFRR*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop front left: ~A ~A ~A ~A" *MFFL* PFFL *MFRR* *RFRR*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "front left stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HFL* (- *HFL* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (2 (let ((step 1) (flag 0) (PFFL 0) (PFRR 0))
                        (ros::spin-once)
                        (setf PFFL (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                        (setf PFRR (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                        (while (< step 16)
                        (send *excelsus* :rear-right-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop rear right: ~A ~A" *RFFL* *RFRR*)
                        ;     (if (and (> *MFFL* (+ *RFFL* *THR*)) (> *MFRR* (+ *RFRR* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFRR* (+ *RFRR* *THR*)) (> *MFFL* (+ PFFL *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFRR *MFRR*)
                                    ; (setf PFFL *MFFL*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop rear right: ~A ~A ~A ~A" *MFRR* PFRR *MFFL* *RFFL*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "rear right stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HRR* (- *HRR* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (3 (let ((step 1) (flag 0) (PFFR 0) (PFRL 0))
                        (ros::spin-once)
                        (setf PFFR (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                        (setf PFRL (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                        (while (< step 16)
                        (send *excelsus* :rear-left-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop rear left: ~A ~A" *RFFR* *RFRL*)
                        ;     (if (and (> *MFFR* (+ *RFFR* *THR*)) (> *MFRL* (+ *RFRL* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFRL* (+ PFRL *THR*)) (> *MFFR* (+ *RFFR* *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFRL *MFRL*)
                                    ; (setf PFFR *MFFR*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop rear left: ~A ~A ~A ~A" *MFRL* PFRL *MFFR* *RFFR*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "rear left stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HRL* (- *HRL* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (t (error "Why is this called?"))
                )
                ;; (send *irtviewer* :draw-objects)
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)
            )
        )

        (setf (elt front-right-pos 0) -130)
        (setf (elt front-right-pos 1) 130)
        (setf (elt front-right-pos 2) -60)
        (setf (elt front-left-pos 0) 130)
        (setf (elt front-left-pos 1) 130)
        (setf (elt front-left-pos 2) -60)
        (setf (elt rear-right-pos 0) -130)
        (setf (elt rear-right-pos 1) -130)
        (setf (elt rear-right-pos 2) -60)
        (setf (elt rear-left-pos 0) 130)
        (setf (elt rear-left-pos 1) -130)
        (setf (elt rear-left-pos 2) -60)

        ;; Reset pose
        (send *excelsus* :fullbody-inverse-kinematics
            (list
                (make-coords :pos front-right-pos)
                (make-coords :pos front-left-pos)
                (make-coords :pos rear-right-pos)
                (make-coords :pos rear-left-pos)
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
        (setf (elt front-right-dif 2) (- *HFR* -60))
        (setf (elt front-left-dif 2) (- *HFL* -60))
        (setf (elt rear-right-dif 2) (- *HRR* -60))
        (setf (elt rear-left-dif 2) (- *HRR* -60))
        (send *excelsus* :front-right-leg :move-end-pos front-right-dif)
        (send *excelsus* :front-left-leg :move-end-pos front-left-dif)
        (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif)
        (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif)
        ; (send *excelsus* :angle-vector #f(46.9764 2.40906 16.1589 -13.7483 46.9762 47.6174 2.62926 16.2842 -13.6534 47.6172 47.8289 3.1044 16.7837 -13.6777 47.8287 47.1906 2.88624 16.6589 -13.7711 -47.1904)) ;; angle-vector of stand up state
        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
        (send *ri* :wait-interpolation)
        (ros::ros-info "STEP END")
        (setf *walk_flag* nil)
    nil )
);; function

;; Function for walking rotation
(defun walk-rotate (mixing order move_leg move_center)
    (setf *walk_flag* t)   
    (ros::ros-info "STEP START")
    (elevate 0)
    (unless (and (= (length mixing) 8) (= (length order) 4))
        (error "Length of inpute vector was mismatched"))
    (let ((startpoint (copy-seq #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0))) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
        (centerpoint #f(0.0 0.0 0.0))
        (endpoint (v+  #f(-130.0 -130.0 130.0 -130.0 -130.0 130.0 130.0 130.0) (scalar-multiply-float-vector mixing move_leg)))
        (front-right-pos #f(0.0 0.0 -60.0))
        (front-left-pos #f(0.0 0.0 -60.0))
        (rear-right-pos #f(0.0 0.0 -60.0))
        (rear-left-pos #f(0.0 0.0 -60.0))
        (front-right-dif #f(0.0 0.0 0.0))
        (front-left-dif #f(0.0 0.0 0.0))
        (rear-right-dif #f(0.0 0.0 0.0))
        (rear-left-dif #f(0.0 0.0 0.0)))
        ;; Move each leg
        (dotimes (i 4)
            (let ((leg_index (elt order i)))
                (setf (elt startpoint (* 2 leg_index)) (elt endpoint (* 2 leg_index))) ;; set x
                (setf (elt startpoint (+ (* 2 leg_index) 1)) (elt endpoint (+ (* 2 leg_index) 1))) ;; set y
                (case leg_index
                    (0 (setf  centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos))) 
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 2) 0))
                    (1 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 1) (+ (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 2) 0))
                    (2 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-right-leg :end-coords :worldpos) (send *excelsus* :rear-left-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (+ (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 2) 0))
                    (3 (setf centerpoint (midpoint 0.5 (send *excelsus* :front-left-leg :end-coords :worldpos) (send *excelsus* :rear-right-leg :end-coords :worldpos)))
                    (setf (elt centerpoint 0) (- (elt centerpoint 0) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 1) (- (elt centerpoint 1) (+ move_center (* 0.4 (* (- i 1) (- i 1) )))))
                    (setf (elt centerpoint 2) 0))
                    (t (error "Why is this called?"))
                ) ;; set center point and this would need to change --------------------------------------------------------------------
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
                    :cog-translation-axis :nil
                    :root-link-virtual-joint-weight #f(100.0 100.0 100.0 0 0 0)
                )

                (case leg_index
                    (0 (setf (elt front-right-dif 2) *LU*) (setf (elt front-left-dif 2) (- *HFL* -60)) (setf (elt rear-right-dif 2) (- *HRR* -60)) (setf (elt rear-left-dif 2) (- *HRR* -60)) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HFR* (- *LU* 60)))
                    (1 (setf (elt front-right-dif 2) (- *HFR* -60)) (setf (elt front-left-dif 2) *LU*) (setf (elt rear-right-dif 2) (- *HRR* -60)) (setf (elt rear-left-dif 2) (- *HRR* -60)) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HFL* (- *LU* 60)))
                    (2 (setf (elt front-right-dif 2) (- *HFR* -60)) (setf (elt front-left-dif 2) (- *HFL* -60)) (setf (elt rear-right-dif 2) *LU*) (setf (elt rear-left-dif 2) (- *HRR* -60)) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HRR* (- *LU* 60)))
                    (3 (setf (elt front-right-dif 2) (- *HFR* -60)) (setf (elt front-left-dif 2) (- *HFL* -60)) (setf (elt rear-right-dif 2) (- *HRR* -60)) (setf (elt rear-left-dif 2) *LU*) (send *excelsus* :front-right-leg :move-end-pos front-right-dif) (send *excelsus* :front-left-leg :move-end-pos front-left-dif) (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif) (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif) (setf *HRL* (- *LU* 60)))
                    (t (error "Why is this called?"))
                )

                ;; (send *irtviewer* :draw-objects)
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)

                (case leg_index
                    (0 (let ((step 1) (flag 0) (PFFR 0) (PFRL 0))
                        (ros::spin-once)
                        (setf PFFR (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                        (setf PFRL (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                        (while (< step 16)
                        (send *excelsus* :front-right-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop front right: ~A ~A" *RFFR* *RFRL*)
                        ;     (if (and (> *MFFR* (+ *RFFR* *THR*)) (> *MFRL* (+ *RFRL* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFFR* (+ PFFR *THR*)) (> *MFRL* (+ *RFRL* *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFFR *MFFR*)
                                    ; (setf PFRL *MFRL*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop front right: ~A ~A ~A ~A" *MFFR* PFFR *MFRL* *RFRL*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "front right stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HFR* (- *HFR* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (1 (let ((step 1) (flag 0) (PFFL 0) (PFRR 0))
                        (ros::spin-once)
                        (setf PFFL (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                        (setf PFRR (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                        (while (< step 16)
                        (send *excelsus* :front-left-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop front left: ~A ~A" *RFFL* *RFRR*)
                        ;     (if (and (> *MFFL* (+ *RFFL* *THR*)) (> *MFRR* (+ *RFRR* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFFL* (+ PFFL *THR*)) (> *MFRR* (+ *RFRR* *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFFL *MFFL*)
                                    ; (setf PFRR *MFRR*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop front left: ~A ~A ~A ~A" *MFFL* PFFL *MFRR* *RFRR*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "front left stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HFL* (- *HFL* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (2 (let ((step 1) (flag 0) (PFFL 0) (PFRR 0))
                        (ros::spin-once)
                        (setf PFFL (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                        (setf PFRR (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                        (while (< step 16)
                        (send *excelsus* :rear-right-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop rear right: ~A ~A" *RFFL* *RFRR*)
                        ;     (if (and (> *MFFL* (+ *RFFL* *THR*)) (> *MFRR* (+ *RFRR* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFRR* (+ *RFRR* *THR*)) (> *MFFL* (+ PFFL *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFRR *MFRR*)
                                    ; (setf PFFL *MFFL*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop rear right: ~A ~A ~A ~A" *MFRR* PFRR *MFFL* *RFFL*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "rear right stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HRR* (- *HRR* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (3 (let ((step 1) (flag 0) (PFFR 0) (PFRL 0))
                        (ros::spin-once)
                        (setf PFFR (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                        (setf PFRL (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                        (while (< step 16)
                        (send *excelsus* :rear-left-leg :move-end-pos #f(0 0 -5))
                        ;; (send *irtviewer* :draw-objects)
                        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                        (send *ri* :wait-interpolation)
                        ( dotimes ( i 10)
                            (ros::spin-once)
                            (setf *MFFR* (max (elt *FFR* 0) (elt *FFR* 1) (elt *FFR* 2) (elt *FFR* 3)))
                            (setf *MFFL* (max (elt *FFL* 0) (elt *FFL* 1) (elt *FFL* 2) (elt *FFL* 3)))
                            (setf *MFRR* (max (elt *FRR* 0) (elt *FRR* 1) (elt *FRR* 2) (elt *FRR* 3)))
                            (setf *MFRL* (max (elt *FRL* 0) (elt *FRL* 1) (elt *FRL* 2) (elt *FRL* 3)))
                            ; (ros::ros-info "reference pressure of legs in sub roop rear left: ~A ~A" *RFFR* *RFRL*)
                        ;     (if (and (> *MFFR* (+ *RFFR* *THR*)) (> *MFRL* (+ *RFRL* *THR*)))
                        ;         (progn ;(setf step (+ step 100))
                        ;         (setf flag (+ flag 1))
                        ;     )
                        ; )
                            (if (and (> *MFRL* (+ PFRL *THR*)) (> *MFFR* (+ *RFFR* *THR*)))
                                (progn
                                    (setf flag (+ flag 1))
                                )
                                (progn (setf PFRL *MFRL*)
                                    ; (setf PFFR *MFFR*)
                                )
                            )
                        )
                        (ros::ros-info "pressure of legs in sub roop rear left: ~A ~A ~A ~A" *MFRL* PFRL *MFFR* *RFFR*)
                        (if (> flag 6)
                            (progn (setf step (+ step 100))
                            (ros::ros-info "rear left stomped ~A" step)))
                        ; (ros::ros-info "height of legs in sub roop: ~A ~A ~A ~A" *HFR* *HFL* *HRR* *HRL*)
                        (setf *HRL* (- *HRL* 5))
                        (setf step (+ step 1)))
                        )
                    )
                    (t (error "Why is this called?"))
                )
                ;; (send *irtviewer* :draw-objects)
                (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
                (send *ri* :wait-interpolation)
            )
        )

        (setf (elt front-right-pos 0) -130)
        (setf (elt front-right-pos 1) 130)
        (setf (elt front-right-pos 2) -60)
        (setf (elt front-left-pos 0) 130)
        (setf (elt front-left-pos 1) 130)
        (setf (elt front-left-pos 2) -60)
        (setf (elt rear-right-pos 0) -130)
        (setf (elt rear-right-pos 1) -130)
        (setf (elt rear-right-pos 2) -60)
        (setf (elt rear-left-pos 0) 130)
        (setf (elt rear-left-pos 1) -130)
        (setf (elt rear-left-pos 2) -60)

        ;; Reset pose
        (send *excelsus* :fullbody-inverse-kinematics
            (list
                (make-coords :pos front-right-pos)
                (make-coords :pos front-left-pos)
                (make-coords :pos rear-right-pos)
                (make-coords :pos rear-left-pos)
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
        (setf (elt front-right-dif 2) (- *HFR* -60))
        (setf (elt front-left-dif 2) (- *HFL* -60))
        (setf (elt rear-right-dif 2) (- *HRR* -60))
        (setf (elt rear-left-dif 2) (- *HRR* -60))
        (send *excelsus* :front-right-leg :move-end-pos front-right-dif)
        (send *excelsus* :front-left-leg :move-end-pos front-left-dif)
        (send *excelsus* :rear-right-leg :move-end-pos rear-right-dif)
        (send *excelsus* :rear-left-leg :move-end-pos rear-left-dif)
        ; (send *excelsus* :angle-vector #f(46.9764 2.40906 16.1589 -13.7483 46.9762 47.6174 2.62926 16.2842 -13.6534 47.6172 47.8289 3.1044 16.7837 -13.6777 47.8287 47.1906 2.88624 16.6589 -13.7711 -47.1904)) ;; angle-vector of stand up state
        (send *ri* :angle-vector (send *excelsus* :angle-vector) *duaration* :default-controller 0 :min-time 0.05)
        (send *ri* :wait-interpolation)
        (ros::ros-info "STEP END")
        (setf *walk_flag* nil)
    nil )
);; function

;; Function for elevate body
(defun elevate (height)
    (setf *walk_flag* t)
    (let ((front-right-pos #f(-130.0 -130.0 0.0)) (front-left-pos #f(130.0 -130.0 0.0)) (rear-right-pos #f(-130.0 130.0 0.0)) (rear-left-pos #f(130.0 130.0 0.0)) (h-ave '(0.0 0.0 0.0 0.0)) offset)
        (setf (elt h-ave 0) *HFR*)
        (setf (elt h-ave 1) *HFL*)
        (setf (elt h-ave 2) *HRR*)
        (setf (elt h-ave 3) *HRL*)
        (setf offset (- -60.0 (average h-ave))) ;; Keep the height of the torso constant.
        (setf (elt front-right-pos 2) (- *HFR* (- height offset)))
        (setf (elt front-left-pos 2) (- *HFL* (- height offset)))
        (setf (elt rear-right-pos 2) (- *HRR* (- height offset)))
        (setf (elt rear-left-pos 2) (- *HRL* (- height offset)))
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
        ;; Update height of legs
        (setf *HFR* (elt front-right-pos 2))
        (setf *HFL* (elt front-left-pos 2))
        (setf *HRR* (elt rear-right-pos 2))
        (setf *HRL* (elt rear-left-pos 2))
        (setf *walk_flag* nil)
    )
);; function

;; Functions for walking toward specified direction
(defun walk-right ()
    (walk #f(-1.0 0.0 -1.0 0.0 -1.0 0.0 -1.0 0.0) (list 0 2 3 1) 32 6) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
);; function

(defun walk-left ()
    (walk #f(1.0 0.0 1.0 0.0 1.0 0.0 1.0 0.0) (list 3 1 0 2) 32 6) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
);; function

(defun walk-forward ()
    (walk #f(0.0 -1.0 0.0 -1.0 0.0 -1.0 0.0 -1.0) (list 1 0 2 3) 30 4) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
);; function

(defun walk-backward ()
    (walk #f(0.0 1.0 0.0 1.0 0.0 1.0 0.0 1.0) (list 2 3 1 0) 30 4) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
);; function

(defun walk-rotate-right ()
    (walk-rotate #f(-47.6 82.4 -82.4 -47.6 82.4 47.6 47.6 -82.4) (list 0 2 3 1) 0.4 4) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
);; function

(defun walk-rotate-left ()
    (walk-rotate #f(82.4 -47.6 47.6 82.4 -47.6 -82.4 -82.4 47.6) (list 0 2 3 1) 0.4 4) ;; (FR_X FR_Y FL_X FL_Y RR_X RR_Y RL_X RL_Y)
);; function

;; Functions for specified actions
(defun walk-right-until-key ()
    (do-until-key
        (walk-right)
    )
);; function
(defun walk-left-until-key ()
    (do-until-key
        (walk-left)
    )
);; function
(defun walk-forward-until-key ()
    (do-until-key
        (walk-forward)
    )
);; function
(defun walk-backward-until-key ()
    (do-until-key
        (walk-backward)
    )
);; function
(defun walk-rotate-right-until-key ()
    (do-until-key
        (walk-rotate-right)
    )
);; function
(defun walk-rotate-left-until-key ()
    (do-until-key
        (walk-rotate-left)
    )
);; function

;; Callback function for retrive command input
(defun cmd_cb (msg)
    (if (not *walk_flag*)
        (progn
            (ros::ros-info "walk-function was called")
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
    )
);; function

;; Callback function for recording value of foot sensors

;; Front Right
(defun stompfr0_cb (msg)
    (setf (elt *FFR* 0) (send msg :wrench :force :x))
);; function
(defun stompfr1_cb (msg)
    (setf (elt *FFR* 1) (send msg :wrench :force :x))
);; function
(defun stompfr2_cb (msg)
    (setf (elt *FFR* 2) (send msg :wrench :force :x))
);; function
(defun stompfr3_cb (msg)
    (setf (elt *FFR* 3) (send msg :wrench :force :x))
);; function

;; Front Left
(defun stompfl0_cb (msg)
    (setf (elt *FFL* 0) (send msg :wrench :force :x))
);; function
(defun stompfl1_cb (msg)
    (setf (elt *FFL* 1) (send msg :wrench :force :x))
);; function
(defun stompfl2_cb (msg)
    (setf (elt *FFL* 2) (send msg :wrench :force :x))
);; function
(defun stompfl3_cb (msg)
    (setf (elt *FFL* 3) (send msg :wrench :force :x))
);; function

;; Rear Right
(defun stomprr0_cb (msg)
    (setf (elt *FRR* 0) (send msg :wrench :force :x))
);; function
(defun stomprr1_cb (msg)
    (setf (elt *FRR* 1) (send msg :wrench :force :x))
);; function
(defun stomprr2_cb (msg)
    (setf (elt *FRR* 2) (send msg :wrench :force :x))
);; function
(defun stomprr3_cb (msg)
    (setf (elt *FRR* 3) (send msg :wrench :force :x))
);; function

;; Rear Left
(defun stomprl0_cb (msg)
    (setf (elt *FRL* 0) (send msg :wrench :force :x))
);; function
(defun stomprl1_cb (msg)
    (setf (elt *FRL* 1) (send msg :wrench :force :x))
);; function
(defun stomprl2_cb (msg)
    (setf (elt *FRL* 2) (send msg :wrench :force :x))
);; function
(defun stomprl3_cb (msg)
    (setf (elt *FRL* 3) (send msg :wrench :force :x))
);; function


;; main process ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; initialize excelsus simulation and robot interface
(excelsus-init)
(ros::ros-info "Excelsus is now initialized")

;; activate servo motors
(on)

;; Wait for initialize servo moter
(ros::duration-sleep 0.2)
(ros::ros-info "Servo Motor was activated ")

;; Initialize subscriber for cmd_vel
(ros::subscribe *cmd_topic* geometry_msgs::Twist #'cmd_cb )

;; Initialize subscriber for force sensor of front right leg
(ros::subscribe *stomp_FR0* geometry_msgs::WrenchStamped #'stompfr0_cb )
(ros::subscribe *stomp_FR1* geometry_msgs::WrenchStamped #'stompfr1_cb )
(ros::subscribe *stomp_FR2* geometry_msgs::WrenchStamped #'stompfr2_cb )
(ros::subscribe *stomp_FR3* geometry_msgs::WrenchStamped #'stompfr3_cb )

;; Initialize subscriber for force sensor of front left leg
(ros::subscribe *stomp_FL0* geometry_msgs::WrenchStamped #'stompfl0_cb )
(ros::subscribe *stomp_FL1* geometry_msgs::WrenchStamped #'stompfl1_cb )
(ros::subscribe *stomp_FL2* geometry_msgs::WrenchStamped #'stompfl2_cb )
(ros::subscribe *stomp_FL3* geometry_msgs::WrenchStamped #'stompfl3_cb )

;; Initialize subscriber for force sensor of rear right leg
(ros::subscribe *stomp_RR0* geometry_msgs::WrenchStamped #'stomprr0_cb )
(ros::subscribe *stomp_RR1* geometry_msgs::WrenchStamped #'stomprr1_cb )
(ros::subscribe *stomp_RR2* geometry_msgs::WrenchStamped #'stomprr2_cb )
(ros::subscribe *stomp_RR3* geometry_msgs::WrenchStamped #'stomprr3_cb )

;; Initialize subscriber for force sensor of rear left leg
(ros::subscribe *stomp_RL0* geometry_msgs::WrenchStamped #'stomprl0_cb )
(ros::subscribe *stomp_RL1* geometry_msgs::WrenchStamped #'stomprl1_cb )
(ros::subscribe *stomp_RL2* geometry_msgs::WrenchStamped #'stomprl2_cb )
(ros::subscribe *stomp_RL3* geometry_msgs::WrenchStamped #'stomprl3_cb )

;; Take initial position
(stand-up)
(ros::ros-info "Excelsus take initial position")

;; Main loo@
(do-until-key
    ;; read sensor value once
    (ros::spin-once)
    (ros::sleep)
)