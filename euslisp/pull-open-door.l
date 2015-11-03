(load "make-door.l")
(defvar *door-radius* 730)
(defvar *knob-radius* 65)
(defvar *prepare-pose* #f(50.1274 -4.84164 18.1891 40.9723 -74.983 199.074 -81.4121 229.895 -21.8931 47.338 14.0862 -121.542 732.86 -114.592 342.337 -0.363463 40.0))


(defun init-door-model ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *door-manip-cds* (make-cascoords :pos (send (send *pr2* :larm :end-coords :copy-worldcoords) :worldpos)))
  (setq *knob-cds* (make-cascoords :pos (send (send *pr2* :larm :end-coords :copy-worldcoords) :worldpos)))
  (setq *tgt-cds* (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
  (setq *door* (instance door :init *door-radius* (elt (send *knob-cds* :worldpos) 2) *knob-radius*))
  (send (car (send *door* :links)) :locate (scale -1 (send *door* :end-coords :worldpos)))
  (send (car (send *door* :links)) :locate (send *pr2* :larm :end-coords :worldpos))
  (send *door-manip-cds* :assoc *knob-cds*)
  (send *knob-cds* :assoc *tgt-cds*)
  (objects (list *pr2* *door*)))

(defun rotate-knob ()
  (let ((tgt-cds (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
        (d (/ 45.0 4))
        avs)
    (send (send *door* :end-coords) :assoc tgt-cds)
    (dotimes (i 4)
      (send *door* :angle-vector (float-vector 0 (- (* (+ i 1) d))))
      
            
    ;;   (send *knob-cds* :translate (float-vector 0 (- *knob-radius*) 0.0))
    ;;   (send *knob-cds* :rotate (deg2rad (- d)) :x)
    ;;   (send *knob-cds* :translate (float-vector 0 *knob-radius* 0.0))
    ;;   (push (send *tgt-cds* :copy-worldcoords) tgt-cds-list))
    ;; (dolist (cds tgt-cds-list)
      (push (send *pr2* :inverse-kinematics tgt-cds
                  :move-target (send *pr2* :larm :end-coords)
                  :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                  :use-torso 0.0)
            avs))
    (send *irtviewer* :draw-objects :flush t)
    (send *ri* :angle-vector-sequence (reverse avs) (list 1000))
    (send *ri* :wait-interpolation)

(defun open-door ()
  (let* ((tgt-cds (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
         (d (/ 100.0 10))
         (base-final-cds (get-base-final-cds))
         avs
         base-cds-list
         knob-base-trans
         (p0 (send (send *pr2* :copy-worldcoords) :worldpos))
         (p1 (send (send base-final-cds :translate #f(100 0 0)) :worldpos)))
   
    ;; (dotimes (i 10)
    ;;   (push (make-coords :pos (midpoint (/ (+ i 1) 10.0) p0 p1) :rot (send (send *pr2* :copy-worldcoords) :worldrot))
    ;;         base-cds-list))
    ;; (setq base-cds-list (reverse base-cds-list))
    (setq knob-base-trans (send (make-coords :pos (send (send *door* :end-coords :copy-worldcoords) :worldpos)) :transformation (send *pr2* :copy-worldcoords)))
    (send (send *door* :end-coords) :assoc tgt-cds)

    ;;calcuate base way-point
    (let* ((base-now (send *pr2* :copy-worldcoords))
           (av-now (send *pr2* :angle-vector))
           (base-tgt (send base-now :copy-worldcoords))
           knob-pos-cds 
           res)
      (dotimes (i 10) ;; angle of door = i * 10
        (send *door* :hinge :joint-angle (- (* (+ i 1) d)))
        (setq knob-pos-cds (send (make-coords :pos (send *door* :end-coords :worldpos) :rot (send base-tgt :worldrot)) :copy-worldcoords))
        (dotimes (j 3)
          (setq base-tgt (make-coords
                         :pos (float-vector
                               (elt (send (send (send knob-pos-cds :copy-worldcoords) :transform knob-base-trans) :pos) 0)
                               (elt (send (send (send knob-pos-cds :copy-worldcoords) :transform knob-base-trans) :pos) 1)
                               0.0)
                         :rot (send knob-pos-cds :copy-rot)))

          (when (= i 1) (send *door* :knob :joint-angle -20))
          (when (= i 2) (send *door* :knob :joint-angle 0))
          (send *pr2* :move-to base-tgt :world)
          (setq res (send *pr2* :inverse-kinematics tgt-cds
                          :move-target (send *pr2* :larm :end-coords)
                          :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                          :use-torso 0.0))
          (if res
              (progn (push res avs) 
                     (push (send base-tgt :copy-worldcoords) base-cds-list)
                     (send *irtviewer* :draw-objects :flush t)
                     (return)))
          (send knob-pos-cds :rotate (deg2rad -5.0) :z :world)
          (send *irtviewer* :draw-objects :flush t))
        (unless res
          (return-from open-door)))

      (send *pr2* :angle-vector av-now)
      (send *pr2* :move-to base-now :world)

    (execute-door-motion (reverse avs) (reverse base-cds-list))
    )))

(defun execute-door-motion (avs base-cds-list)
  (let ((t-interval 1000.0) ;msec
        traj-points
        diff-v
        diff-cds
        sttm)
    
    (dotimes (i (length base-cds-list))
      ;; (if (= i 0)
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

    (send *ri* :angle-vector-sequence avs t-interval :default-controller sttm)
    (send *ri* :move-trajectory-sequence 
          (reverse traj-points)
          (make-list (length base-cds-list) :initial-element t-interval)
          :start-time sttm
          :send-action t)
    ))

    ;; (send (send *door* :end-coords) :assoc tgt-cds)
    ;; (dotimes (i 10)
    ;;   ;; (send *pr2* :move-to (elt base-cds-list i) :world)
    ;;   (send *door* :hinge :joint-angle (- (* (+ i 1) d)))
    ;;   (unless (send *pr2* :inverse-kinematics tgt-cds
    ;;               :move-target (send *pr2* :larm :end-coords)
    ;;               :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
    ;;               :use-torso 0.0)
    ;;     (return nil))
    ;;   (push (send *pr2* :angle-vector) avs)
    ;;   (send *irtviewer* :draw-objects :flush t))
      


(defun get-base-final-cds ()
  (let* ((rec (send *door* :angle-vector))
         (base-final-cds (send *pr2* :copy-worldcoords))
         knob-0-cds
         knob-90-cds
         diff)
    (send *door* :angle-vector #f(0 0))
    (setq knob-0-cds (send *door* :end-coords :copy-worldcoords))
    (send *door* :angle-vector #f(-90 0))
    (setq knob-90-cds (send *door* :end-coords :copy-worldcoords))
    (setq diff (v- (send knob-90-cds :worldpos) (send knob-0-cds :worldpos)))
    
    (send base-final-cds :translate diff :world)
    (send *door* :angle-vector rec)
    base-final-cds))







    

    

        

        
  


