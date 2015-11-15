(load "models/room73b2-door-left-object.l")
(load "models/room73b2-door-right-object.l")

(setq *door-fix* nil)
(setq *door-rotate* nil)
(setq *knob-cds* nil)
(setq *use-arm* :larm)

(defun init-scene
  ()
  (if (boundp '*pr2*)
      (progn
        (send *pr2* :move-to (make-coords) :world)
        (send *pr2* :reset-pose))
    (pr2))
  (if (not (boundp '*irtviewer*))
    (make-irtviewer))
  (setq *door-fix* (room73b2-door-left))
  (setq *door-rotate* (room73b2-door-right))
  (send *door-fix* :translate #f(1000 405.0 0))
  (send *door-rotate* :translate #f(1000 -405.0 0))
  (send (send *door-rotate* :handle-front-knob) :translate #f(-175 0 0) :world)
  ;;init knob-cds
  (setq *knob-cds* (make-cascoords :coords (send *door-rotate* :handle-front-knob :copy-worldcoords)))
  (send *knob-cds* :rotate (deg2rad 180) :z)
  (send (send *door-rotate* :handle-front-knob) :assoc *knob-cds*)
  (send *irtviewer* :objects (list *pr2* *door-fix* *door-rotate*))
  )

(defun adjust-pose-to-grasp-knob
  (&optional (knob-cds *knob-cds*))
  ;; (let (base-target-cds (make-coords :pos (float-vector (elt (send knob-cds :worldpos) 0)
  ;;                                                       (elt (send knob-cds :worldpos) 1)
  ;;                                                       0.0))))
  (send *pr2* :translate #f(300 -405 0))
  )

(defun grasp-knob
  (&optional (knob-cds *knob-cds*))
  (let ((pre-cds (send (send knob-cds :copy-worldcoords) :translate #f(-80 0 0))))
    (send *pr2* :inverse-kinematics (send (send *knob-cds* :copy-worldcoords) :translate #f(-80 0 0))
          :move-target (send *pr2* *use-arm* :end-coords)
          :link-list (send *pr2* :link-list (send *pr2* *use-arm* :end-coords :parent))
          :use-torso 0.01
          :use-base t
          :base-range (list :min #f(-150 0 0) :max #f(100 0 0)))
          

    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    (send *ri* :stop-grasp :larm)
    
    (send *pr2* :inverse-kinematics knob-cds
          :move-target (send *pr2* *use-arm* :end-coords)
          :link-list (send *pr2* :link-list (send *pr2* *use-arm* :end-coords :parent))
          :use-torso 0.0)

    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
    ))



(defun rotate-knob 
  (&optional (knob-cds *knob-cds*) &key (real-robot t))
  (when real-robot
    (send *pr2* :angle-vector (send *ri* :potentio-vector)))
  (let ((tgt-cds (make-cascoords :coords (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
        (d (/ 45.0 4))
        avs)
    (send knob-cds :assoc tgt-cds)
    (dotimes (i 4)
      (send *door-rotate* :knob :joint-angle (* (+ i 1) d))
      (push (send *pr2* :inverse-kinematics tgt-cds
                  :move-target (send *pr2* *use-arm* :end-coords)
                  :link-list (send *pr2* :link-list (send *pr2* *use-arm* :end-coords :parent))
                  :use-torso 0.0)
            avs))
    (send *irtviewer* :draw-objects :flush t)
    (when real-robot
        (send *ri* :angle-vector-sequence (reverse avs) (list 1000))
        (send *ri* :wait-interpolation))
    ))

(defun door-base-collide
  ()
   (pqp-collision-check-objects (list (car (send *pr2* :torso :parent-link))) (append (send *door-fix* :links) (send *door-rotate* :links)))
  )



(defun door-arm-collision-link-pair ()
  (let ((ls1 (send *pr2* *use-arm* :links))
        (ls2 (send *door-fix* :links))
        res)
    (dolist (l1 ls1)
      (dolist (l2 ls2)
        (push (list l1 l2) res)))
    res))

(defun door-base-collision-link-pair ()
  (let ((bl (car (send *pr2* :torso :parent-link)))
        (dfls (send *door-fix* :links))
        (drls (send *door-rotate* :links))
        res)
    (dolist (dfl dfls)
      (push (list bl dfl) res))
    (dolist (drl drls)
      (push (list bl drl) res))
    res))
    


(defun generate-motion 
  (&key (real-robot t))
  (when real-robot
    (send *pr2* :angle-vector (send *ri* :potentio-vector)))
  (let* ((tgt-cds (make-cascoords :coords (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
         (d (/ 100.0 10))
         ;; avs
         ;; base-cds-list
         (p0 (send (send *pr2* :copy-worldcoords) :worldpos))
         (p1 (send (send *pr2* :copy-worldcoords) :translate #f(1000 0 0)))
         (st-av (send *pr2* :angle-vector))
         (st-base-cds (send *pr2* :copy-worldcoords))
         rotated-angle
         pos-diff)

    (setq avs nil)
    (setq base-cds-list nil) ;;debug
    (send *knob-cds* :assoc tgt-cds)
    
    (dotimes (i 10)
      (send *door-rotate* :door :joint-angle (* i 10))
      (when (= i 2) (send *door-rotate* :knob :joint-angle 20))
      (when (= i 3) (send *door-rotate* :knob :joint-angle 0))
      (setq rotated-angle (vector-angle (matrix-column (send st-base-cds :worldrot) 0) (matrix-column (send *pr2* :copy-rot) 0) #f(0 0 1)))
      ;; (setq pos-diff (v- (send *pr2* :copy-pos) (send st-base-cds :worldpos)))
      
      (send *pr2* :move-to (make-coords :pos (send *pr2* :copy-pos) :rot (send st-base-cds :copy-rot)) :world)
      (print rotated-angle)

      (send tgt-cds :draw-on :flush t :size 100 :color #f(0 0 1))
      (send *pr2* :inverse-kinematics tgt-cds
            :move-target (send *pr2* *use-arm* :end-coords)
            :link-list (send *pr2* :link-list (send *pr2* *use-arm* :end-coords :parent))
            :collision-avoidance-link-pair (door-arm-collision-link-pair)
            :avoid-collision-distance 200
            :avoid-collision-joint-gain 1.0
            :avoid-collision-null-gain 80.0
            :rotation-axis :z
            :use-torso t
            :use-base t
            :base-range (list :min (float-vector -150 0 -30) 
                              :max (float-vector 300 0  30))
            :debug-view nil
            :additional-check #'(lambda () (not (door-base-collide))))
      (push (send *pr2* :copy-worldcoords) base-cds-list)
      (push (send *pr2* :angle-vector) avs)
      ;;        (send *pr2* :translate #f(100 0 0) :world))
      )
    (send *pr2* :angle-vector st-av)
    (send *pr2* :move-to st-base-cds :world)
    (list (reverse avs) (reverse base-cds-list))
    ))

(defun execute-push-door-motion (avs base-cds-list)
  (let ((t-interval 1000.0) ;msec
        traj-points
        diff-v
        diff-cds
        sttm)
    

    (dotimes (i (length base-cds-list))
      ;; (if (= i 0)
      (print i)
          (setq diff-cds (send (send *pr2* :copy-worldcoords) :transformation (elt base-cds-list i)))
        ;; (setq diff-cds (send (elt base-cds-list (- i 1)) :transformation (elt base-cds-list i))))
          (setq diff-v (send diff-cds :pos))
          (push (float-vector (elt diff-v 0)
                              (elt diff-v 1)
                              (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1)))
                              traj-points))
    (setq sttm (ros::time+ (ros::time-now) (ros::time 2)))
    (setq *traj-points* traj-points)
    (setq *base-cds-list* base-cds-list)
    (setq *avs* avs)
    (send *ri* :angle-vector-sequence avs (list t-interval) :default-controller sttm)
    (send *ri* :move-trajectory-sequence 
          (reverse traj-points)
          (make-list (length base-cds-list) :initial-element t-interval)
          :start-time sttm
          :send-action t)
    ))

(defun test ()
  (init-scene)
  (if (eq *use-arm* :larm)
      (send *pr2* :rarm :angle-vector (cadr *pr2-tuckarm-pose-larm-free-outside*))
    (send *pr2* :larm :angle-vector (car *pr2-tuckarm-pose-rarm-free-outside*)))
  (adjust-pose-to-grasp-knob)
  (grasp-knob)
  (rotate-knob *knob-cds* :real-robot nil)
  (generate-motion :real-robot nil))

(defun play-motion (avs base-cds-list)
  (let ((recent-av (send *pr2* :angle-vector))
        (recent-cds (send *pr2* :copy-worldcoords)))
  (dotimes (i (length avs))
    (print (door-base-collide))
    (send *door-rotate* :door :joint-angle (* i 10))
    (send *pr2* :move-to (elt base-cds-list i) :world)
    (unix::usleep (* 1000 500))
    (send *pr2* :angle-vector (elt avs i))
    (send *irtviewer* :draw-objects :flush t))
  (send *pr2* :angle-vector recent-av)
  (send *pr2* :move-to recent-cds :world)))
  
  
(defun demo ()
  (rotate-knob *knob-cds* :real-robot t)
  (let ((motion (generate-motion)))
    (play-motion (car motion) (cadr motion))
    (execute-push-door-motion (car motion) (cadr motion)))
  )
