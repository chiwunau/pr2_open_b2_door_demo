(load "package://drc_tasks/euslisp/make-door.l")
(defvar *door-radius* 730)
(defvar *knob-radius* 65)


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
    (send *ri* :angle-vector-sequence (reverse avs) (list 1000))))

(defun open-door ()
  (let* ((tgt-cds (make-cascoords :coords (send *pr2* :larm :end-coords :copy-worldcoords)))
        (d (/ 100.0 10))
        (base-final-cds (get-base-final-cds))
        avs
        base-cds-list
        (p0 (send (send *pr2* :copy-worldcoords) :worldpos))
        (p1 (send (send base-final-cds :translate #f(100 0 0)) :worldpos)))
   
    (dotimes (i 10)
      (push (make-coords :pos (midpoint (/ (+ i 1) 10.0) p0 p1) :rot (send (send *pr2* :copy-worldcoords) :worldrot))
            base-cds-list))
    (setq base-cds-list (reverse base-cds-list))
    
    (send (send *door* :end-coords) :assoc tgt-cds)
    (dotimes (i 10)
      (send *pr2* :move-to (elt base-cds-list i) :world)
      (send *door* :hinge :joint-angle (- (* (+ i 1) d)))
      (unless (send *pr2* :inverse-kinematics tgt-cds
                  :move-target (send *pr2* :larm :end-coords)
                  :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
                  :use-torso 0.0)
        (return nil))
      (push (send *pr2* :angle-vector) avs)
      (send *irtviewer* :draw-objects :flush t)
      (read)
      )))


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
    
    
    



    

    

        

        
  


