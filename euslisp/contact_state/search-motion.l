(unless (boundp '*pr2*)
  (load "package://pr2eus/pr2-interface.l"))
(require "make-door.l")

(setq *left-samples* nil)
(setq *right-samples* nil)
(setq *filtered-left-samples* nil)
(setq *filtered-right-samples* nil)
(setq debugview-colli nil)
(setq debugview-mintq nil)





(defclass motion-samples
  :slots (pose-seq base-seq torque-seq))
(defmethod motion-samples
  (:init
   ()
   )

  (:push
   (name data)
   (cond ((eq name :pose-seq)
          (push data pose-seq))
         ((eq name :base-seq)
          (push data base-seq))
         ((eq name :torque-seq)
          (push data torque-seq))))

  (:get
   (name)
   (cond ((eq name :pose-seq)
          pose-seq)
         ((eq name :base-seq)
          base-seq)
         ((eq name :torque-seq)
          torque-seq)))
  )
;; (setq *sampled-motion-sequence* nil)
;; (setq *sampled-base-cds-sequence* nil)
;; (setq *sampled-torque-sequence* nil)
(unless (boundp '*use-arm*)
  (setq *use-arm* :larm))

(setq *contact-pts* nil)

(defun init()
  (pr2)
  (objects *pr2*)
  (make-door)
  (objects *door*)
  (setq *left-samples* (instance motion-samples :init))
  (setq *right-samples* (instance motion-samples :init)))


(defun reset-scene ()
  (send *pr2* :move-to (make-coords) :world)
  (send *door* :hinge :joint-angle 0)
  (set-contact-points)
  (setq *left-samples* (instance motion-samples :init))
  (setq *right-samples* (instance motion-samples :init)))
  )

(defun set-contact-points ()
  (setq *contact-pts* nil)
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
  (let ((tgt-cds-list (reverse (send *door* :get-rear-touch-cds-list)))
        pose-list
        base-cds-list
        torque-list
        pose-list-seq
        base-cds-list-seq
        torque-list-seq
        (org-base-cds (send *pr2* :copy-worldcoords))
        res
        tgt-cds)
    
    (dolist (arm (list :rarm :larm))
;;    (dolist (arm (list :larm))
      (setq *use-arm* arm)
      (set-contact-points)
      (dolist (contact-pt *contact-pts*)
        (setq torque-list-seq nil)
        (setq pose-list-seq nil)
        (setq base-cds-list-seq nil)
        (dotimes (i 5)
          (setq torque-list nil)
          (setq pose-list nil)
          (setq base-cds-list nil)
          (send *door* :hinge :joint-angle (* (+ i 1) -20))

          (dolist (cds tgt-cds-list)
            (cond (
                   ;;-----------------FOR END-EFFECTOR--------------------------
                   (eq contact-pt (elt *contact-pts* 0))
                   (setq tgt-cds (send (send cds :copy-worldcoords) :translate #f(50 0 0))) ;;shift from cds
                   (setq res (send *pr2* :inverse-kinematics tgt-cds
                                   :move-target contact-pt ;;(send *pr2* *use-arm* :end-coords)
                                   :link-list (send *pr2* :link-list (send contact-pt :parent)) ;;(send *pr2* *use-arm* :end-coords :parent))
                                   :collision-avoidance-link-pair (door-arm-collision-link-pair contact-pt)
                                   :avoid-collision-distance 50
                                   :avoid-collision-joint-gain 1.0
                                   :avoid-collision-null-gain 100.0
                                   :rotation-axis nil;;:z
                                   :use-torso t
                                   :use-base 0.5
                                   :base-range (list :min (float-vector -500 -500 -90)
                                                     :max (float-vector 500 500  90))
                                   ;; :base-range (list :min (float-vector -150 0 -30)
                                   ;;                   :max (float-vector 300 0  30))
                                   :warnp nil
                                   :debug-view debugview-colli
                                   :additional-check #'(lambda () (not (door-base-collide)))
                                   ))
                   (when res ;;when collision with door clear, search motion that satisfy torque limit
                     (setq res (send *pr2* :inverse-kinematics tgt-cds
                                     :move-target contact-pt
                                     :link-list (send *pr2* :link-list (send contact-pt :parent))
                                     :null-space #'(lambda () 
                                                     (let* ((drw (door-reaction-wrench cds contact-pt))
                                                           (z (calc-torque-norm-gradient (car drw) (cadr drw) (send contact-pt :copy-worldcoords) :link-list (send *pr2* :link-list (send contact-pt :parent)) :move-target contact-pt :use-base t)))
                                                       (scale -0.0 z)))
                                     :rotation-axis nil
                                     :use-torso t
                                     :use-base t
                                     :warnp nil
                                     :debug-view debugview-mintq
                                     :additional-check #'(lambda ()
                                                           (let* ((max-torque (send *pr2* :max-torque-vector))
                                                                  (overload nil)
                                                                  (drw (door-reaction-wrench cds contact-pt))
                                                                  (torque (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords  (list contact-pt)))
                                                                  )
                                                             (dotimes (i (length max-torque))
                                                               (if (> 0 (- (abs (elt max-torque i)) (abs (elt torque i))))
                                                                   (setq overload t)))
                                                             (if (not overload) (print "All joint torque limit satisfied!"))
                                                             (not overload)))
                                     )))
                   )

                  ;;-----------------FOR FOREARM AND ELBOW--------------------------
                  ((or (eq contact-pt (elt *contact-pts* 1)) (eq contact-pt (elt *contact-pts* 2)))
                   (setq tgt-cds (send (send (send cds :copy-worldcoords) :translate #f(50 0 0))  :rotate (deg2rad 90) :y))
                   (setq res (send *pr2* :inverse-kinematics tgt-cds
                                   :move-target contact-pt ;;(send *pr2* *use-arm* :end-coords)
                                   :link-list (send *pr2* :link-list (send contact-pt :parent)) ;;(send *pr2* *use-arm* :end-coords :parent))
                                   :collision-avoidance-link-pair (door-arm-collision-link-pair contact-pt)
                                   :avoid-collision-distance 50
                                   :avoid-collision-joint-gain 1.0
                                   :avoid-collision-null-gain 100.0
                                   :rotation-axis (if (eq contact-pt (elt *contact-pts* 1)) :z nil) ;;:z for forearm nil for elbow 
                                   :use-torso t
                                   :use-base 1.0
                                   :base-range (list :min (float-vector -1000 -1000 -90)
                                                     :max (float-vector 1000 1000  90))
                                   ;; :base-range (list :min (float-vector -150 0 -30)
                                   ;;                   :max (float-vector 300 0  30))
                                   :warnp nil
                                   :debug-view debugview-colli
                                   :additional-check #'(lambda () (not (door-base-collide))))
                         )
                   (when res
                     (setq res (send *pr2* :inverse-kinematics tgt-cds
                                     :move-target contact-pt
                                     :link-list (send *pr2* :link-list (send contact-pt :parent))
                                     :null-space #'(lambda ()
                                                     (let* ((drw (door-reaction-wrench cds contact-pt))
                                                           (z (calc-torque-norm-gradient (car drw) (cadr drw) (send contact-pt :copy-worldcoords) :link-list (send *pr2* :link-list (send contact-pt :parent)) :move-target contact-pt :use-base t)))
                                                       (scale -0.005 z)))
                                     :rotation-axis nil
                                     :use-torso t
                                     :use-base t
                                     :warnp nil
                                     :debug-view debugview-mintq
                                     :additional-check #'(lambda ()
                                                           (let* ((max-torque (send *pr2* :max-torque-vector))
                                                                  (overload nil)
                                                                  (drw (door-reaction-wrench cds contact-pt))
                                                                  (torque (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords  (list contact-pt)))
                                                                  )
                                                             (dotimes (i (length max-torque))
                                                               (if (> 0 (- (abs (elt max-torque i)) (abs (elt torque i))))
                                                                   (setq overload t)))
                                                             (if (not overload) (print "All joint torque limit satisfied!"))
                                                             (not overload)))
                                     )))
                   )
                  )
            
            (let* ((drw (door-reaction-wrench cds contact-pt)))
              (push (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords  (list contact-pt)) torque-list)
              (push (send *pr2* :copy-worldcoords) base-cds-list)
              (push res pose-list)
              )
            (send *pr2* :move-to org-base-cds :world))
          (push (reverse torque-list) torque-list-seq)
          (push (reverse base-cds-list) base-cds-list-seq)
          (push (reverse pose-list) pose-list-seq)
          )
        ;; (push (reverse torque-list-seq) *sampled-torque-sequence*)
        ;; (push (reverse base-cds-list-seq) *sampled-base-cds-sequence*)
        (send (if (eq *use-arm* :rarm) *right-samples* *left-samples*) :push :torque-seq (reverse torque-list-seq))
        (send (if (eq *use-arm* :rarm) *right-samples* *left-samples*) :push :base-seq (reverse base-cds-list-seq))
        (send (if (eq *use-arm* :rarm) *right-samples* *left-samples*) :push :pose-seq (reverse pose-list-seq))
        ))))


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

(defun door-reaction-wrench (cds-on-door-plane contact-cds)
  (let ((cds cds-on-door-plane)
        (tau (* 2.5 9.8 700)) ;;2.5kg from handle
        (moment #f(0 0 0))
        (target-cds contact-cds)
        force
        dist-to-hinge
        res)
    
    (setq dist-to-hinge (abs (elt (send (send cds :transformation (send (send *door* :hinge :parent-link) :worldcoords)) :pos) 1)))
    (setq force (float-vector (/ tau dist-to-hinge) 0 0)) ;;force in door plane cds
    (setq force (send target-cds :inverse-rotate-vector (send cds :rotate-vector force))) ;;force in contact-cds
    (list force moment)
    )
  )

;; (defun get-hold-door-torque (cds-on-door-plane contact-cds)
;;   (let ((cds cds-on-door-plane)
;;         (tau (* 2.5 9.8 700)) ;;just any number, no real data
;;         (moment #f(0 0 0))
;;         (target-cds contact-cds)
;;         force
;;         dist-to-hinge
;;         res)
    
;;     (setq dist-to-hinge (abs (elt (send (send cds :transformation (send (send *door* :hinge :parent-link) :worldcoords)) :pos) 1)))
;;     (setq force (float-vector (/ tau dist-to-hinge) 0 0)) ;;force in door plane cds
;;     (setq force (send target-cds :inverse-transform-vector (send cds :transform-vector force))) ;;force in contact-cds
;;     (send *pr2* :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords  (list target-cds))
;;     )
;;   )

(setq *play-speed* 50)
(defun play-motion
  (&key (motion nil) (filter nil))
  (if motion
      (dotimes (i (length (car motion)))
        (if (elt (elt motion 0) i)
            (progn
              (send *door* :hinge :joint-angle (* (+ i 1) -20))
              (send *pr2* :angle-vector (elt (elt motion 0) i))
              (send *pr2* :move-to (elt (elt motion 1) i) :world)
              (send *irtviewer* :draw-objects)
              (send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 5 (elt (elt motion 2) i)))
              (print (norm (elt (elt motion 2) i)))
              (send *irtviewer* :viewer :viewsurface :flush)
               (print (pqp-collision-check *pr2* *door*))
              ))
        (unix::usleep (* 1000 *play-speed*)))
    ;;else
    (dolist (arm (list :rarm :larm))
      (let* ((samples (if filter (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) (if (eq arm :rarm) *right-samples* *left-samples*)))
             (pose-list-seq (send samples :get :pose-seq))
             (base-cds-list-seq (send samples :get :base-seq))
             (torque-list-seq (send samples :get :torque-seq)))
        (dotimes (i (length pose-list-seq))
          (dotimes (j (length (elt pose-list-seq i)))
            (send *door* :hinge :joint-angle (* (+ j 1) -20))
            (dotimes (k (length (elt (elt pose-list-seq i) j)))
              (unless (eq (elt (elt (elt pose-list-seq i) j) k) nil)
                (send *pr2* :angle-vector (elt (elt (elt pose-list-seq i) j) k))
                (send *pr2* :move-to (elt (elt (elt base-cds-list-seq i) j) k) :world)
                (send *irtviewer* :draw-objects)
                (send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 5 (elt (elt (elt torque-list-seq i) j) k)))
                (send *irtviewer* :viewer :viewsurface :flush)
                (print (calc-pose-torque-overshoot (elt (elt (elt torque-list-seq i) j) k)))
                (unix::usleep (* 1000 50)))
              )))))
    ))

(defun get-random-motion
  (&optional (arm :rarm) (idx 0))
  (let* ((pose-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :pose-seq) idx))
         (base-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :base-seq) idx))
         (torque-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :torque-seq) idx))
         motion
         pl bl tl)

    (dotimes (i (length pose-list)) ;;door angle
      (if (elt pose-list i)
          (progn
            (let ((n (random (length (elt pose-list i)))))
              (push (elt (elt pose-list i) n) pl)
              (push (elt (elt base-list i) n) bl)
              (push (elt (elt torque-list i) n) tl)))
        (progn
          (push nil pl)
          (push nil bl)
          (push nil tl))))
    (setq motion (list (reverse pl) (reverse bl) (reverse tl)))
    motion))

(defun filter-samples ;;filter out pose that collide with door
  ()
  (setq *filtered-left-samples* (instance motion-samples :init))
  (setq *filtered-right-samples* (instance motion-samples :init))

  (dolist (arm (list :rarm :larm))
    (let ((pose-list-seq (send (if (eq arm :rarm) *right-samples* *left-samples*) :get :pose-seq))
          (base-cds-list-seq (send (if (eq arm :rarm) *right-samples* *left-samples*) :get :base-seq))
          (torque-list-seq (send (if (eq arm :rarm) *right-samples* *left-samples*) :get :torque-seq))
          filtered-pose-list-seq filtered-base-list-seq filtered-torque-list-seq)

      (dotimes (i (length pose-list-seq)) ;;number of contact points
        (let (pose-list base-list torque-list)
          (dotimes (j (length (elt pose-list-seq i))) ;;number of door angle
            (let (tmplp tmplb tmplt)
              (send *door* :hinge :joint-angle (* (+ j 1) -20))
              (dotimes (k (length (elt (elt pose-list-seq i) j)))
                (send *pr2* :angle-vector (elt (elt (elt pose-list-seq i) j) k)) ;;tmp
                (send *pr2* :move-to (elt (elt (elt base-cds-list-seq i) j) k) :world) ;;tmp
                (if (and (elt (elt (elt pose-list-seq i) j) k) (not (pqp-collision-check-objects (send *pr2* arm :links)  (send *door* :links))))
                    (progn
                      (push (elt (elt (elt pose-list-seq i) j) k) tmplp)
                      (push (elt (elt (elt base-cds-list-seq i) j) k) tmplb)
                      (push (elt (elt (elt torque-list-seq i) j) k) tmplt))))
              (push (reverse tmplp) pose-list)
              (push (reverse tmplb) base-list)
              (push (reverse tmplt) torque-list)))
        (push (reverse pose-list) filtered-pose-list-seq)
        (push (reverse base-list) filtered-base-list-seq)
        (push (reverse torque-list) filtered-torque-list-seq)))

      (dolist (l filtered-pose-list-seq)
        (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :push :pose-seq l))
      (dolist (l filtered-base-list-seq)
        (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :push :base-seq l))
      (dolist (l filtered-torque-list-seq)
        (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :push :torque-seq l)))))



      

;;(dotimes (r 2) (dotimes (i 3) (play-motion (get-random-motion (if (= r 0) :rarm :larm) i)))) 
;;make-door
;;set-contact-points
;;motion-generation
;;play-motion
