(setq *sampled-motion-sequence* nil)
(setq *sampled-base-cds-sequence* nil)
(setq *use-arm* :rarm)


(defun motion-generation ()
  (let ((tgt-cds-list (send *door* :get-rear-touch-cds-list))
        motion-list
        res)
    (dolist (cds tgt-cds-list)
      (setq res (send *pr2* :rarm :inverse-kinematics cds :rotation-axis nil :debug-view :no-message))
      (push res motion-list))
  (push motion-list *sampled-motion-sequence*)))

(defun motion-generation-new ()
  (let ((tgt-cds-list (reverse (send *door* :get-rear-touch-cds-list)))
        motion-list
        base-cds-list
        (org-base-cds (send *pr2* :copy-worldcoords))
        res)
    (dotimes (i 4)
      (setq motion-list nil)
      (setq base-cds-list nil)
      (send *door* :hinge :joint-angle (* (+ i 1) -20))
    (dolist (cds tgt-cds-list)
      (setq res (send *pr2* :inverse-kinematics cds
                      :move-target (send *pr2* *use-arm* :end-coords)
                      :link-list (send *pr2* :link-list (send *pr2* *use-arm* :end-coords :parent))
                      :collision-avoidance-link-pair (door-arm-collision-link-pair)
                      :avoid-collision-distance 100
                      :avoid-collision-joint-gain 1.0
                      :avoid-collision-null-gain 100.0
                      :rotation-axis nil
                      :use-torso t
                      :use-base 1.0
                      :base-range (list :min (float-vector -500 -500 -30)
                                        :max (float-vector 500 500  30))
                      ;; :base-range (list :min (float-vector -150 0 -30)
                      ;;                   :max (float-vector 300 0  30))
                      :debug-view nil
                      :additional-check #'(lambda () (not (door-base-collide))))
            )
      (push (send *pr2* :copy-worldcoords) base-cds-list)
      (push res motion-list)
      (send *pr2* :move-to org-base-cds :world))
    (push base-cds-list *sampled-base-cds-sequence*)
    (push motion-list *sampled-motion-sequence*)
    )))


(defun door-arm-collide
  ()
  (pqp-collision-check-objects (send *pr2* *use-arm* :links) (send *door* :links))
  )

(defun door-base-collide
  ()
   (pqp-collision-check-objects (list (car (send *pr2* :torso :parent-link))) (send *door* :links))
  )

(defun door-arm-collision-link-pair ()
  (let ((ls1 (reverse (cddr (reverse (send *pr2* *use-arm* :links)))))
        (ls2 (send *door* :links))
        res)
    (dolist (l1 ls1)
      (dolist (l2 ls2)
        (push (list l1 l2) res)))
    res))

(defun play-motion
  ()
  (let ((motion-list (reverse *sampled-motion-sequence*))
        (base-cds-list (reverse *sampled-base-cds-sequence*)))
    (dotimes (i (length motion-list))
      (send *door* :hinge :joint-angle (* (+ i 1) -20))
      (dotimes (j (length (elt motion-list i)))
        (send *pr2* :angle-vector (elt (elt motion-list i) j))
        (send *pr2* :move-to (elt (elt base-cds-list i) j) :world)
      (send *irtviewer* :draw-objects)))
    ))
   
   
    
