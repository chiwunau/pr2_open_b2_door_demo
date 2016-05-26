(unless (boundp '*pr2*)
  (load "package://pr2eus/pr2-interface.l")
  (pr2))
(setq *sampled-motion-sequence* nil)
(setq *sampled-base-cds-sequence* nil)
(setq *sampled-torque-sequence* nil)
(unless (boundp '*use-arm*)
  (setq *use-arm* :rarm))

(setq *contact-pts* nil)

(defun set-contact-points ()
  (let (pt0 pt1 pt2 pt3)
    (setq pt0 (make-cascoords :coords (send *pr2* *use-arm* :end-coords :copy-worldcoords)))
    (send (send *pr2* *use-arm* :end-coords :parent) :assoc pt0)

    (setq pt1 (make-cascoords :coords (send (elt (send *pr2* *use-arm* :links) 4) :copy-worldcoords))) ;;r_forearm_roll_link
    (send pt1 :translate #f(200 0 0))
    (send (elt (send *pr2* *use-arm* :links) 4) :assoc pt1)

    (setq pt2 (make-cascoords :coords (send (elt (send *pr2* *use-arm* :links) 3) :copy-worldcoords)))
    (send pt2 :translate #f(-70 0 -70))
    (send (elt (send *pr2* *use-arm* :links) 3) :assoc pt2)

    (push pt0 *contact-pts*)
    (push pt1 *contact-pts*)
    (push pt2 *contact-pts*)
    (setq *contact-pts* (reverse *contact-pts*))
    ))

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
        torque-list
        motion-list-seq
        base-cds-list-seq
        torque-list-seq
        (org-base-cds (send *pr2* :copy-worldcoords))
        res)
    (dolist (contact-pt *contact-pts*)
      (setq torque-list-seq nil)
      (setq motion-list-seq nil)
      (setq base-cds-list-seq nil)
      (dotimes (i 5)
        (setq torque-list nil)
        (setq motion-list nil)
        (setq base-cds-list nil)
        (send *door* :hinge :joint-angle (* (+ i 1) -20))

        (dolist (cds tgt-cds-list)
          (cond ((eq contact-pt (elt *contact-pts* 0))
                 (setq res (send *pr2* :inverse-kinematics (send (send cds :copy-worldcoords) :translate #f(20 0 0));;cds
                                 :move-target contact-pt ;;(send *pr2* *use-arm* :end-coords)
                                 :link-list (send *pr2* :link-list (send contact-pt :parent)) ;;(send *pr2* *use-arm* :end-coords :parent))
                                 :collision-avoidance-link-pair (door-arm-collision-link-pair contact-pt)
                                 :avoid-collision-distance 100
                                 :avoid-collision-joint-gain 1.0
                                 :avoid-collision-null-gain 100.0
                                 :rotation-axis nil;;:z
                                 :use-torso t
                                 :use-base 1.0
                                 :base-range (list :min (float-vector -1000 -1000 -90)
                                                   :max (float-vector 1000 1000  90))
                                 ;; :base-range (list :min (float-vector -150 0 -30)
                                 ;;                   :max (float-vector 300 0  30))
                                 :debug-view nil ;;:no-message
                                 ;;                      :additional-check #'(lambda () (not (door-base-collide)))
                                 )
                       )
                 )
                ;;-----------------FOR FOREARM AND ELBOW--------------------------
                ((or (eq contact-pt (elt *contact-pts* 1)) (eq contact-pt (elt *contact-pts* 2)))
                 (setq res (send *pr2* :inverse-kinematics (send (send (send cds :copy-worldcoords) :translate #f(50 0 0))  :rotate (deg2rad 90) :y);;cds
                                 :move-target contact-pt ;;(send *pr2* *use-arm* :end-coords)
                                 :link-list (send *pr2* :link-list (send contact-pt :parent)) ;;(send *pr2* *use-arm* :end-coords :parent))
                                 :collision-avoidance-link-pair (door-arm-collision-link-pair contact-pt)
                                 :avoid-collision-distance 100
                                 :avoid-collision-joint-gain 1.0
                                 :avoid-collision-null-gain 100.0
                                 :rotation-axis (if (eq contact-pt (elt *contact-pts* 1)) :z nil) ;;:z for forearm nil for elbow 
                                 :use-torso t
                                 :use-base 1.0
                                 :base-range (list :min (float-vector -1000 -1000 -90)
                                                   :max (float-vector 1000 1000  90))
                                 ;; :base-range (list :min (float-vector -150 0 -30)
                                 ;;                   :max (float-vector 300 0  30))
                                 :debug-view nil ;;:no-message
                                 :additional-check #'(lambda () (not (door-base-collide))))
                       )
                 )
                )
          (push (get-hold-door-torque cds contact-pt) torque-list)
          (push (send *pr2* :copy-worldcoords) base-cds-list)
          (push res motion-list)
          (send *pr2* :move-to org-base-cds :world))
        (push torque-list torque-list-seq)
        (push base-cds-list base-cds-list-seq)
        (push motion-list motion-list-seq)
        )
      (push (reverse torque-list-seq) *sampled-torque-sequence*)
      (push (reverse base-cds-list-seq) *sampled-base-cds-sequence*)
      (push (reverse motion-list-seq) *sampled-motion-sequence*)
      )
      )))


(defun door-arm-collide
  ()
  (pqp-collision-check-objects (send *pr2* *use-arm* :links) (send *door* :links))
  )

(defun door-base-collide
  ()
   (pqp-collision-check-objects (list (car (send *pr2* :torso :parent-link))) (send *door* :links))
  )

(defun door-arm-collision-link-pair (contact-pt)
  (let ((ls1 (send *door* :links))
        (ls2 (send *pr2* :link-list (send contact-pt :parent)))
        res)
    (dolist (l1 ls1)
      (push (list (send *pr2* :base_footprint_lk) l1) res)
      (dolist (l2 ls2)
        (push (list l2 l1) res))) ;;robot-link obstacle
    res))

(defun get-hold-door-torque (cds-on-door-plane contact-cds)
  (let ((cds cds-on-door-plane)
        (tau 1000.0) ;;just any number, no real data
        (moment #f(0 0 0))
        (target-cds contact-cds)
        force
        dist-to-hinge
        res)
    
    (setq dist-to-hinge (abs (elt (send (send cds :transformation (send (send *door* :hinge :parent-link) :worldcoords)) :pos) 1)))
    (setq force (float-vector (/ tau dist-to-hinge) 0 0)) ;;force in door plane cds
    (setq force (send target-cds :inverse-transform-vector (send cds :transform-vector force))) ;;force in contact-cds
    (send *pr2* :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords  (list target-cds))
    )
  )



(defun play-motion
  ()
  (let ((motion-list-seq (reverse *sampled-motion-sequence*))
        (base-cds-list-seq (reverse *sampled-base-cds-sequence*))
        (torque-list-seq (reverse *sampled-torque-sequence*)))
    (dotimes (i (length motion-list-seq))
      (dotimes (j (length (elt motion-list-seq i)))
        (send *door* :hinge :joint-angle (* (+ j 1) -20))
        (dotimes (k (length (elt (elt motion-list-seq i) j)))
          (unless (eq (elt (elt (elt motion-list-seq i) j) k) nil)
            (send *pr2* :angle-vector (elt (elt (elt motion-list-seq i) j) k))
            (send *pr2* :move-to (elt (elt (elt base-cds-list-seq i) j) k) :world)
            (send *irtviewer* :draw-objects)
            (send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 10 (elt (elt (elt torque-list-seq i) j) k)))
            (send *irtviewer* :viewer :viewsurface :flush)
            (unix::usleep (* 1000 200)))
          )))
    ))
   
   
    
