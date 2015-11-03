(load "models/room73b2-door-left-object.l")
(load "models/room73b2-door-right-object.l")

(setq *door-fix* (room73b2-door-left))
(setq *door-rotate* (room73b2-door-right))
(setq *knob-cds* nil)
(setq *use-arm* :rarm)

(defun init-scene
  ()
  (if (boundp '*pr2*)
      (send *pr2* :move-to (make-coords) :world)
    (pr2))
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
  (knob-cds)
  ;; (let (base-target-cds (make-coords :pos (float-vector (elt (send knob-cds :worldpos) 0)
  ;;                                                       (elt (send knob-cds :worldpos) 1)
  ;;                                                       0.0))))
  (send *pr2* :translate #f(350 -405 0))
  )

(defun grasp-knob
  (knob-cds)
  (let ((pre-cds (send (send knob-cds :copy-worldcoords) :translate #f(-80 0 0))))
    (send *pr2* *use-arm* :inverse-kinematics pre-cds :look-at-target t)
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector 4000))
    ;;(send *ri* :stop-grasp :larm)
    
    (send *pr2* *use-arm* :inverse-kinematics knob-cds :look-at-target t)
    ;;(send *ri* :angle-vector (send *pr2* :angle-vector 4000))
    ))

(defun door-base-collide
  ()
  (pqp-collision-check-objects (list (send (car (send *pr2* :torso :joint)) :parent-link)) (send *door-rotate* :links)))


(defun rotate-knob 
  (knob-cds)
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
    ;; (send *ri* :angle-vector-sequence (reverse avs) (list 1000))
    ;; (send *ri* :wait-interpolation)
    ))

      


(defun generate-motion ()
  (let* ((tgt-cds (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
         (d (/ 100.0 10))
         (base-final-cds (get-base-final-cds))
         avs
         base-cds-list
         knob-base-trans
         (p0 (send (send *pr2* :copy-worldcoords) :worldpos))
         (p1 (send (send *pr2* :copy-worldcoords) :translate #f(1000 0 0))))

    (send *knob-cds* :assoc tgt-cds)
    
    (dotimes (i 10)
      (send 
            
    


    
        
  



