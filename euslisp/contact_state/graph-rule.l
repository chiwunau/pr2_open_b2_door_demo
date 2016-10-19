(load "~/ros/hydro/src/jsk-ros-pkg/euslib/rbrain/virtual_interpolator.l")

(setq *cof-p* 0.7)
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


(defun overload-node? (n &optional (door-weight 2.5))
  (let* ((force-cds (elt (send n :get :cds-info) 0))
         ;;             (contact-pt (elt (send n :get :cds-info) 2))
         (contact-pt (elt (reverse (set-contact-points (send n :get :arm))) (send n :get :contact-point)))
         (max-tq (send *pr2* :max-torque-vector))
         (overload nil)
         drw tq)
    (send *pr2* :angle-vector (send n :get :pose))
    (send *pr2* :move-to (send n :get :base) :world)
    (send *door* :hinge :joint-angle (send n :get :door-angle))
    
    (setq drw (door-reaction-wrench force-cds contact-pt door-weight))
    (setq tq (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords (list contact-pt)))
    
    (dotimes (i (length max-tq))
      (if (> 0 (- (abs (elt max-tq i)) (abs (elt tq i))))
          (setq overload t)))

    overload))

(defun transitable? (n1 n2)
  (let* ((mid-pose (cadr (cadr (pos-list-interpolation (list (send n1 :get :pose) (send n2 :get :pose)) (list 1000) 500))))
         (diff-cds (send (send (send n1 :get :base) :transformation (send n2 :get :base)) :copy-worldcoords))
         (mid-cds (send (send (send (send n1 :get :base) :copy-worldcoords) :translate (scale 0.5 (send diff-cds :pos))) :rotate (* 0.5 (caar (send diff-cds :rpy-angle))) :z))
         (mid-door-angle (/ (+ (send n1 :get :door-angle) (send n2 :get :door-angle)) 2)))

    (send *pr2* :angle-vector mid-pose)
    (send *pr2* :move-to mid-cds :world)
    (send *door* :hinge :joint-angle mid-door-angle)
    t
   (not (pqp-collision-check-objects (append (send *pr2* :torso :parent-link) (send *pr2* (send n1 :get :arm) :links))  (send *door* :links)))
    ))


;;copy from /irteus/test
;; (defun pos-list-interpolation
;;   (pos-list ;; (list pos_1 pos_2 ... pos_N), pos_i is float-vector
;;    time-list ;; (list dtime_1 dtime_2 ... dtime_{N-1}), dtime_i is time[s] between time at pos_{i+1} - pos_i
;;    dt ;; dt [s]                              
;;    &key (interpolator-class minjerk-interpolator)
;;    ((:interpolator ip) (instance interpolator-class :init))
;;    (initial-time 0.0) (neglect-first) (vel-vector-list) (acc-vector-list))
;;   (let* ((data-list) (tm-list) (vel-data-list) (acc-data-list))
;;     (send* ip :reset
;;            :position-list pos-list
;;            :time-list (let (r) (dolist (n time-list) (push (+ n (if r (car r) 0)) r)) (nreverse r)) ;; list of time[sec] from start for each control point
;;            (append
;;             (if vel-vector-list (list :vel-vector-list vel-vector-list))
;;             (if acc-vector-list (list :acc-vector-list acc-vector-list))))
;;     (send ip :start-interpolation)
;;     (while (send ip :interpolatingp)
;;       (push (if (send ip :interpolatingp)
;;                 (+ initial-time (send ip :time))
;;               (+ dt (car tm-list))) tm-list)
;;       (send ip :pass-time dt)
;;       (push (send ip :position) data-list)
;;       (if (find-method ip :vel) (push (send ip :vel) vel-data-list))
;;       (if (find-method ip :acc) (push (send ip :acc) acc-data-list))
;;       )
;;     (append
;;      (list :data (if neglect-first (cdr (reverse data-list)) (reverse data-list))
;;            :time (if neglect-first (cdr (reverse tm-list)) (reverse tm-list)))
;;      (if (find-method ip :vel)
;;          (list :vel (if neglect-first (cdr (reverse vel-data-list)) (reverse vel-data-list))))
;;      (if (find-method ip :acc)
;;          (list :acc (if neglect-first (cdr (reverse acc-data-list)) (reverse acc-data-list))))
;;      )))

