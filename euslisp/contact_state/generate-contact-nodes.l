(load "extended-graph.l")
(load "graph-rule.l")

(setq *all-nodes* nil)
(setq *solvable-nodes* nil)
(setq *door-rot-step* 20)
(setq *graph-depth* 2)
(setq *ik-target-shift* #f(50 0 0)) ;;;target coords of IK shift from door plane
(setq debugview-colli nil)
(setq debugview-mintq nil)
;; (setq debugview-colli :no-message)
;; (setq debugview-mintq :no-message)



(defun set-contact-points (&optional arm)
  (setq *contact-pts* nil)
  (let (pt0 pt1 pt2 pt3)
    (setq pt0 (make-cascoords :coords (send *pr2* arm :end-coords :copy-worldcoords)))
    (send (send *pr2* arm :end-coords :parent) :assoc pt0)
    
    (setq pt1 (make-cascoords :coords (send (elt (send *pr2* arm :links) 4) :copy-worldcoords))) ;;r_forearm_roll_link
    (send pt1 :translate #f(200 0 0))
    (send (elt (send *pr2* arm :links) 4) :assoc pt1)

    (setq pt2 (make-cascoords :coords (send (elt (send *pr2* arm :links) 3) :copy-worldcoords)))
    (send pt2 :translate #f(-70 0 -70))
    (send (elt (send *pr2* arm :links) 2) :assoc pt2) ;;; previously 3, 2 is supposedly correct

    (push pt0 *contact-pts*)
    (push pt1 *contact-pts*)
    (push pt2 *contact-pts*)
    (setq *contact-pts* (reverse *contact-pts*))
    ))

(defun init-all-container-nodes ()
  (setq *all-nodes* nil)
  (dolist (arm (list :rarm :larm))
    (let ((robot-contact-pts (set-contact-points arm))
          (obj-cds-list (send *door* :get-rear-touch-cds-list))
          (graph-depth *graph-depth*))
          (dotimes (robot-contact-pts-idx (length robot-contact-pts))
            (dotimes (depth graph-depth)
;;;-------------------point/line contact nodes-----------------------------;;;
              (dotimes (obj-cds-idx (length obj-cds-list))
                (let ((n (instance extended-node :init (format nil "~A-~A-~A-~A"
                                                      (if (equal arm :rarm) 'r 'l) robot-contact-pts-idx depth obj-cds-idx))))
                  
                  (send n :put-value :arm arm)
                  (send n :put-value :contact-point-index robot-contact-pts-idx)
                  (send n :put-value :contact-point (elt robot-contact-pts robot-contact-pts-idx))
                  (send n :put-value :contact-type (case robot-contact-pts-idx (1 'line) (t 'point)))
                  (send n :put-value :graph-depth depth)
                  (send n :put-value :door-angle (* (+ depth 1) (* -1 *door-rot-step*)))
                  ;; ;                  (send n :put-value :pose-idx k)
                  (send n :put-value :solved nil)
                  (send n :put-value :robot-pose nil)
                  (send n :put-value :robot-pose-optimized nil)
                  (send n :put-value :base-coords nil)
                  (send n :put-value :base-coords-optimized nil)
                  (send n :put-value :object-coords-index obj-cds-idx)
                  (send n :put-value :object-coords (elt obj-cds-list obj-cds-idx))
                  (send n :put-value :ik-target-coords (send (elt obj-cds-list obj-cds-idx) :assoc (make-cascoords :coords (send (send (send (elt obj-cds-list obj-cds-idx) :copy-worldcoords) :translate *ik-target-shift*) :rotate (if (eq robot-contact-pts-idx 0) 0 pi/2) :y))))
                  (send n :put-value :torque nil)
                  (send n :put-value :overload? nil)
                  
                  (push n *all-nodes*)))
;;;-------------------grasp contact nodes-----------------------------;;;
              (dolist (knob-cds (list (send *door* :front-knob-coords) (send *door* :rear-knob-coords)))
                (when (eq robot-contact-pts-idx 0) ;;end-effector
                  (let ((n (instance extended-node :init (format nil "~A-~A-~A-~A"
                                                                 (if (equal arm :rarm) 'r 'l) robot-contact-pts-idx depth (if (equal (send knob-cds :name) "front-knob-coords") 'fk 'rk)))))
                      
                    (send n :put-value :arm arm)
                    (send n :put-value :contact-point-index robot-contact-pts-idx)
                    (send n :put-value :contact-point (elt robot-contact-pts robot-contact-pts-idx))
                    (send n :put-value :contact-type 'grasp)
                    (send n :put-value :graph-depth depth)
                    (send n :put-value :door-angle (* (+ depth 1) (* -1 *door-rot-step*)))
                    ;; ;                  (send n :put-value :pose-idx k)
                    (send n :put-value :solved nil)
                    (send n :put-value :robot-pose nil)
                    (send n :put-value :robot-pose-optimized nil)
                    (send n :put-value :base-coords nil)
                    (send n :put-value :base-coords-optimized nil)
                    (send n :put-value :object-coords-index (if (equal (send knob-cds :name) "front-knob-coords") 'fk 'rk))
                    (send n :put-value :object-coords knob-cds)
                    (setq *knob-cds* knob-cds)
                    (send n :put-value :ik-target-coords (send knob-cds :assoc (make-cascoords :coords (send (send (send knob-cds :copy-worldcoords) :rotate (if (equal (send knob-cds :name) "front-knob-coords") 0 pi) :z) :rotate pi/2 :x))))
                    (send n :put-value :torque nil)
                    (send n :put-value :overload? nil)
                    (push n *all-nodes*))))
              ))))
  (setq *all-nodes* (reverse *all-nodes*))
  nil)

(defun robot-pose-sampling (&key (nodes *all-nodes*))
  (dolist (n nodes)
    (send *pr2* :move-to (make-coords) :world)
    (send *pr2* :reset-pose)
    (send n :put-value :solved t)
    (send *door* :hinge :joint-angle (send n :get-value :door-angle))
    (let* ((cds (send n :get-value :object-coords))
           (tgt-cds (send n :get-value :ik-target-coords))
           (contact-pt (send n :get-value :contact-point))
           (res (send *pr2* :inverse-kinematics tgt-cds
                      :move-target contact-pt
                      :link-list (send *pr2* :link-list (send contact-pt :parent))
                      :collision-avoidance-link-pair (door-arm-base-collision-link-pair contact-pt :use-base t :include-knob (if (eq (send n :get-value :contact-type) 'grasp) nil t))
                      :avoid-collision-distance 50
                      :avoid-collision-joint-gain 5.0
                      :avoid-collision-null-gain 100.0
                      :rotation-axis (case (send n :get-value :contact-type)
                                           ('point nil)
                                           ('line :z)
                                           ('grasp t)
                                           (t t))

;;                      (if (eq (send n :get-value :contact-point-index) 1) :z nil) ;;:z(line contact) for forearm
                      :use-torso t
                      :use-base t
                      :base-range (list :min (float-vector -600 -600 -80)
                                        :max (float-vector 600 600  80))
                      :warnp nil
                      :debug-view debugview-colli
                      )))
      
      (if (and res (pqp-collision-check-objects 
                    (append (send *pr2* :base_footprint_lk) (send *pr2* (send n :get-value :arm) :links))
                    (if (eq (send n :get-value :contact-type) 'grasp)
                        (remove-if #'(lambda (x) (substringp "knob" (string (send x :name)))) (send *door* :links))
                      (send *door* :links))))
          (setq res nil))
      
      (when res ;;when collision with door clear, search motion that satisfy torque limit
        (send n :put-value :robot-pose (send *pr2* :angle-vector))
        (send n :put-value :base-coords (send *pr2* :copy-worldcoords))

        ;; (setq res (send *pr2* :inverse-kinematics tgt-cds
        ;;                 :move-target contact-pt
        ;;                 :link-list (send *pr2* :link-list (send contact-pt :parent))
        ;;                 ;; :collision-avoidance-link-pair (door-arm-base-collision-link-pair contact-pt :use-base t)
        ;;                 ;; :stop 100
        ;;                 ;; :avoid-collision-distance 50
        ;;                 ;; :avoid-collision-joint-gain 1.0
        ;;                 ;; :avoid-collision-null-gain 100.0
        ;;                 ;; :null-space #'(lambda ()
        ;;                 ;;                 (let* ((drw (door-reaction-wrench cds (make-coords)))
        ;;                 ;;                        (z (calc-torque-norm-gradient (car drw) (cadr drw) ;;(send n :get-value :contact-point)
        ;;                 ;;                                                      tgt-cds
        ;;                 ;;                                                      :link-list (send *pr2* :link-list (send contact-pt :parent)) :move-target contact-pt :use-base t)))
        ;;                 ;;                   ;; (send *irtviewer* :viewer :viewsurface :line-width 4)
        ;;                 ;;                   ;; (send *irtviewer* :viewer :viewsurface :color #f(1 0 0))
        ;;                 ;;                   ;; (send *irtviewer* :viewer :draw-arrow 
        ;;                 ;;                   ;;       (send (send n :get-value :contact-point) :worldpos)
        ;;                 ;;                   ;;       (v+ (send (send n :get-value :contact-point) :worldpos) (scale 10 (car drw))))
        ;;                 ;;                   ;; (send *irtviewer* :viewer :flush)
        ;;                 ;;                   ;;                                                     (print (incf *hoge*))
        ;;                 ;;                   (setq z (normalize-vector z))
        ;;                 ;;                   (scale -0.05 z)))
        ;;               :rotation-axis (case (send n :get-value :contact-type)
        ;;                                    ('point nil)
        ;;                                    ('line :z)
        ;;                                    ('grasp t)
        ;;                                    (t t))
        ;;                 :use-torso t
        ;;                 :use-base t
        ;;                 :warnp nil
        ;;                 :debug-view debugview-mintq
        ;;                 ))
        ;; ;; (when (and res (pqp-collision-check-objects (append (send *pr2* :base_footprint_lk) (send *pr2* (send n :get-value :arm)  :links)) (send *door* :links)))
        ;; (when res
        ;;   (send n :put-value :robot-pose-optimized (send *pr2* :angle-vector))
        ;;   (send n :put-value :base-coords-optimized (send *pr2* :copy-worldcoords))
        ;;    )
        )
      )))

(defun get-solvable-nodes (&key (nodes *all-nodes*))
  (let (rect-nodes)
    (dolist (n nodes)
      (if (and (send n :get-value :solved) (send n :get-value :robot-pose))
          (push n rect-nodes)))
    (setq *solvable-nodes* (reverse rect-nodes))))

(defun get-target-neighbour-nodes (n)
  (let ((idx (send n :get-value :object-coords-index))
        (h (send *door* :h-tgts-num))
        (v (send *door* :v-tgts-num)))
    (print (list idx h v))
    (dolist (nb-idx (list (- idx v 1) (- idx v) (+ (- idx v) 1) (- idx 1) (+ idx 1) (+ idx v -1) (+ idx v) (+ idx v 1)))
            (print nb-idx))))
            
  
  
         

(defun play-node (n &key (optimized nil))
  (send *pr2* :angle-vector (send n :get-value (if optimized :robot-pose-optimized :robot-pose)))
  (send *pr2* :move-to (send n :get-value (if optimized :base-coords-optimized :base-coords)) :world)
  (send *door* :hinge :joint-angle (send n :get-value :door-angle))
  (send *irtviewer* :draw-objects :flush t))

(defun door-arm-base-collision-link-pair (contact-pt &key (use-base t) (include-knob i))
  (let ((ls1 (send *door* :links))
        (ls2 (send *pr2* :link-list (send contact-pt :parent)))
        res)
    (dolist (l1 ls1)
      (if use-base (push (list (send *pr2* :base_footprint_lk) l1) res))
      (unless (and (not include-knob) (or (eq (send l1 :name) 'front-knob) (eq (send l1 :name) 'rear-knob)))
        (dolist (l2 ls2)
          (push (list l2 l1) res)))) ;;robot-link obstacle
      res))


(defun test ()
  (init-all-container-nodes)
  (robot-pose-sampling)
  (filter-unsolved-nodes))
      
;;(dolist (n *all-nodes*) (when (send n :get-value :robot-pose) (progn (play-node n) (unix:usleep (* 1000 500)))))
          
                  
;;;
