(unless (boundp '*pr2*)
  (load "package://pr2eus/pr2-interface.l")
  (pr2))
(setq *sampled-motion-sequence* nil)
(setq *sampled-base-cds-sequence* nil)
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

    (push pt0 *contact-pts*)
    (push pt1 *contact-pts*)
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
        (org-base-cds (send *pr2* :copy-worldcoords))
        (contact-pt (elt *contact-pts* 1))
        res)
    (dotimes (i 5)
      (setq motion-list nil)
      (setq base-cds-list nil)
      (send *door* :hinge :joint-angle (* (+ i 1) -20))
    (dolist (cds tgt-cds-list)
      (setq res (send *pr2* :inverse-kinematics (send (send (send cds :copy-worldcoords) :translate #f(50 0 0))  :rotate (deg2rad 90) :y);;cds
                      :move-target contact-pt ;;(send *pr2* *use-arm* :end-coords)
                      :link-list (send *pr2* :link-list (send contact-pt :parent)) ;;(send *pr2* *use-arm* :end-coords :parent))
                      :collision-avoidance-link-pair (door-arm-collision-link-pair contact-pt)
                      :avoid-collision-distance 100
                      :avoid-collision-joint-gain 1.0
                      :avoid-collision-null-gain 100.0
                      :rotation-axis :z
                      :use-torso t
                      :use-base 1.0
                      :base-range (list :min (float-vector -1000 -1000 -90)
                                        :max (float-vector 1000 1000  90))
                      ;; :base-range (list :min (float-vector -150 0 -30)
                      ;;                   :max (float-vector 300 0  30))
                      :debug-view nil;;:no-message
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

(defun door-arm-collision-link-pair (contact-pt)
  (let ((ls1 (send *pr2* :link-list (send contact-pt :parent)))
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
        (unless (eq (elt (elt motion-list i) j) nil)
          (send *pr2* :angle-vector (elt (elt motion-list i) j))
          (send *pr2* :move-to (elt (elt base-cds-list i) j) :world)
          (send *irtviewer* :draw-objects)
          (unix::usleep (* 1000 200))
          )))
    ))
   
   
    
