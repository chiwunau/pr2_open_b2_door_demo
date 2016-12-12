(setq *cof-p* 0.0)
(defun calc-distance-between-pose
  (p1 p2 b1 b2) ;;pose1(reccent) pose2(next) base1(reccent) base2(next)
  (let* ((cof-p *cof-p*)
         (cof-b (- 1 cof-p))
         (cof-b-trans 1.0)
         (cof-b-rot 1.0)
         (diff-p (v- p2 p1))
         (diff-cds (send (send b1 :transformation b2) :copy-worldcoords)))

    (setf (elt diff-p 0) 0)
    (setq diff-p (deg2rad (norm diff-p)))
    (setq diff-b (+ (* cof-b-trans (/ (norm (send diff-cds :pos)) 1000))
                    (* cof-b-rot (abs (car (car (send diff-cds :rpy-angle)))))))

    ;; (format a "~A~%" diff-p)
    ;; (format b "~A~%" diff-b)
    ;; (format c "~A~%" (/ (norm (send diff-cds :pos)) 1000))
    ;; (format d "~A~%" (abs (car (car (send diff-cds :rpy-angle)))))
    ;; (format t "~A, ~A, " diff-p diff-b)
    ;; (format t "~A, ~A~%" (/ (norm (send diff-cds :pos)) 1000) (abs (car (car (send diff-cds :rpy-angle)))))
    (+ (* cof-p diff-p) (* cof-b diff-b))))


;; (defun overload-node? (n &optional (door-weight 2.5))
;;   (let* ((force-cds (elt (send n :get :cds-info) 0))
;;          ;;             (contact-pt (elt (send n :get :cds-info) 2))
;;          (contact-pt (elt (reverse (set-contact-points (send n :get :arm))) (send n :get :contact-point)))
;;          (max-tq (send *pr2* :max-torque-vector))
;;          (overload nil)
;;          drw tq)
;;     (send *pr2* :angle-vector (send n :get :pose))
;;     (send *pr2* :move-to (send n :get :base) :world)
;;     (send *door* :hinge :joint-angle (send n :get :door-angle))
    
;;     (setq drw (door-reaction-wrench force-cds contact-pt door-weight))
;;     (setq tq (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords (list contact-pt)))
    
;;     (dotimes (i (length max-tq))
;;       (if (> 0 (- (abs (elt max-tq i)) (abs (elt tq i))))
;;           (setq overload t)))

;;     overload))

(defun colide-obstacle? (n)
  (unless *obs*
    (return-from colide-obstacle? nil))
  (if (send n :get-value :robot-pose-optimized)
      (progn
        (send *pr2* :angle-vector (send n :get-value :robot-pose-optimized))
        (send *pr2* :move-to (send n :get-value :base-coords-optimized) :world))
    (progn
      (send *pr2* :angle-vector (send n :get-value :robot-pose))
      (send *pr2* :move-to (send n :get-value :base-coords) :world)))
  (pqp-collision-check-objects 
   (list (send *pr2* :base_footprint_lk))
   (list *obs*))
  )
  
  


;; (defun transitable? (n1 n2)
;;   (let* ((mid-pose (cadr (cadr (pos-list-interpolation (list (send n1 :get-value :pose) (send n2 :get-value :pose)) (list 1000) 500))))
;;          (diff-cds (send (send (send n1 :get-value :base) :transformation (send n2 :get-value :base)) :copy-worldcoords))
;;          (mid-cds (send (send (send (send n1 :get-value :base) :copy-worldcoords) :translate (scale 0.5 (send diff-cds :pos))) :rotate (* 0.5 (caar (send diff-cds :rpy-angle))) :z))
;;          (mid-door-angle (/ (+ (send n1 :get-value :door-angle) (send n2 :get-value :door-angle)) 2)))

;;     (send *pr2* :angle-vector mid-pose)
;;     (send *pr2* :move-to mid-cds :world)
;;     (send *door* :hinge :joint-angle mid-door-angle)
;;     t
;;    (not (pqp-collision-check-objects (append (send *pr2* :torso :parent-link) (send *pr2* (send n1 :get-value :arm) :links))  (send *door* :links)))
;;     ))

(defun transitable? (n1 n2)
(return-from transitable? t) ;;for-debug
  (let* ((pose-pli (pos-list-interpolation (list (send n1 :get-value :pose) (send n2 :get-value :pose)) (list 750) 250))
         (diff-cds (send (send (send n1 :get-value :base) :transformation (send n2 :get-value :base)) :copy-worldcoords))
         (base-pos-rot-v (coerce (append (coerce (send diff-cds :pos) cons) (list (caar (send diff-cds :rpy-angle)))) float-vector))
         (base-pli (pos-list-interpolation (list (coerce (make-list (length base-pos-rot-v) :initial-element 0) float-vector) base-pos-rot-v) (list 750) 250)))
;;         (mid-cds (send (send (send (send n1 :get-value :base) :copy-worldcoords) :translate (scale 0.5 (send diff-cds :pos))) :rotate (* 0.5 (caar (send diff-cds :rpy-angle))) :z))
  ;;       (mid-door-angle (/ (+ (send n1 :get-value :door-angle) (send n2 :get-value :door-angle)) 2)))

    (dotimes (i (length (cadr pose-pli)))
      (let ((mid-cds (send (send (send (send n1 :get-value :base) :copy-worldcoords)
                                 :translate (subseq (elt (cadr base-pli) i) 0 3))
                           :rotate (elt (elt (cadr base-pli) i) 3) :z))
            (mid-door-angle (+ (send n1 :get-value :door-angle) (* i (/ (- (send n2 :get-value :door-angle) (send n1 :get-value :door-angle)) (* 1.0 (- (length (cadr pose-pli)) 1)))))))
        (send *pr2* :angle-vector (elt (cadr pose-pli) i))
        (send *pr2* :move-to mid-cds :world)
        (send *door* :hinge :joint-angle (+ mid-door-angle -3))
        ;; (print (elt (cadr pose-pli) i))
        ;; (print mid-cds)
        ;; (print mid-door-angle)
     ;; (print mid-door-angle)
     ;; (send *irtviewer* :draw-objects)
     ;; (unix::usleep (* 1000 500))
        (let (tmpl) 
          (dolist (dl (send *door* :links)) (unless (string-equal (send dl :name) "rear-knob")
                                                (push dl tmpl)))
          (when (or (pqp-collision-check-objects (append (send *pr2* :torso :parent-link) (send *pr2* (send n1 :get-value :arm) :links)) (send *door* :links)) (check-arm-self-collision (send n1 :get-value :arm)))
            ;; (send *irtviewer* :draw-objects)
            ;; (unix::usleep (* 1000 500))
            (return-from transitable? nil))
          (dotimes (i 90) ;;make sure robot's arm do not collide with rear knob
            (when (pqp-collision-check-objects (send *pr2* (send n1 :get-value :arm)  :links) (list (elt (send *door* :links) (position "rear-knob" (send-all (send *door* :links) :name) :test 'string-equal))))
              (return-from transitable? nil))
            (send *door* :hinge :joint-angle (+ i -90))
;;            (send *irtviewer* :draw-objects)
            )
        )))
    t))

(defun check-arm-self-collision
  (&optional (arm :larm))
  (let (tmpl)
    (dolist (l (send *pr2* :links)) (unless (member l (send *pr2* arm :links)) (push l tmpl)))
    (push (car (send *pr2* :torso :parent-link)) tmpl)
    (pqp-collision-check-objects (cdr (send *pr2* arm :links)) tmpl)))

(defun get-node (name)
  (let ((p (position name (send-all *all-nodes* :name) :test 'equal)))
    (if p
        (elt *all-nodes* p))))

