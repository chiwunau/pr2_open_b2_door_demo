(defun max-smoothness-motion
  (&optional (arm :rarm) (idx 0))
  (let* ((pose-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :pose-seq) idx))
         (base-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :base-seq) idx))
         (torque-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :torque-seq) idx))
         st-p st-b
         motion
         pl bl tl)

    (let ((r (random (length (elt pose-list 0)))))
          (setq st-p (elt (elt pose-list 0) r))
          (setq st-b (elt (elt base-list 0) r)))

    (let ((p st-p)
          (b st-b)
          next-p tmp-p tmp-b min-dist dist p-idx)
      (dotimes (i (length pose-list)) ;;door angle
        (setq min-dist nil)
        (dotimes (j (length (elt pose-list i)))
          (setq tmp-p (elt (elt pose-list i) j))
          (setq tmp-b (elt (elt base-list i) j))
;          (setq dist (v- p tmp-p))
          (setq dist (calc-distance-between-pose p tmp-p b tmp-b))
;;          (setf (elt dist 0) 0)
;;          (setq dist (norm dist))
          (when (or (not min-dist) (< dist min-dist))
            (setq min-dist dist)
            (setq p-idx j)))

        (if (elt (elt pose-list i) p-idx)
            (progn
              (push (elt (elt pose-list i) p-idx) pl)
              (push (elt (elt base-list i) p-idx) bl)
              (push (elt (elt torque-list i) p-idx) tl))
          (progn
            (push nil pl)
            (push nil bl)
            (push nil tl)))
      (setq p (elt (elt pose-list i) p-idx))))

    (setq motion (list (reverse pl) (reverse bl) (reverse tl)))
    motion))

(setq *cof-p* 0.7)
(defun calc-distance-between-pose
  (p1 p2 b1 b2) ;;pose1(reccent) pose2(next) base1(reccent) base2(next)
  (let* ((cof-p *cof-p*)
         (cof-b (- 1 cof-p))
         (cof-b-trans 1.0)
         (cof-b-rot 1.0)
         (diff-p (v- p2 p1))
         (diff-cds (send (send b1 :transformation b2) :copy-worldcoords)))

    (print cof-p)
    (print cof-b)
    (setf (elt diff-p 0) 0)
    (setq diff-p (deg2rad (norm diff-p)))
    (setq diff-b (+ (* cof-b-trans (/ (norm (send diff-cds :pos)) 1000))
                    (* cof-b-rot (abs (car (car (send diff-cds :rpy-angle)))))))

    (format a "~A~%" diff-p)
    (format b "~A~%" diff-b)
    (format c "~A~%" (/ (norm (send diff-cds :pos)) 1000))
    (format d "~A~%" (abs (car (car (send diff-cds :rpy-angle)))))
    ;; (format t "~A, ~A, " diff-p diff-b)
    ;; (format t "~A, ~A~%" (/ (norm (send diff-cds :pos)) 1000) (abs (car (car (send diff-cds :rpy-angle)))))
    (+ (* cof-p diff-p) (* cof-b diff-b))))


(defun calc-pose-torque-overshoot
  (tqv)
  (let* ((max-tqv (send *pr2* :max-torque-vector))
         (overshoot 0))
    (dotimes (i (length max-tqv))
      (let ((tmp (/ (abs (elt tqv i)) (abs (elt max-tqv i)))))
        (if (<= tmp 1)
            (setq overshoot (+ overshoot 0))
          (setq overshoot (+ overshoot tmp)))))
;;    (print overshoot)
    overshoot))

(defun min-torque-norm-motion
  (&optional (arm :rarm) (idx 0))
  (let* ((pose-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :pose-seq) idx))
         (base-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :base-seq) idx))
         (torque-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :torque-seq) idx))
         st-p st-b
         motion
         pl bl tl)

    (let ((r (random (length (elt pose-list 0)))))
          (setq st-p (elt (elt pose-list 0) r))
          (setq st-b (elt (elt pose-list 0) r)))

    (let ((p st-p)
          next-p tmp-p min-torque tmp-t torque p-idx)
      (dotimes (i (length pose-list)) ;;door angle
        (setq min-torque nil)
        (dotimes (j (length (elt pose-list i)))
          (setq tmp-t (elt (elt torque-list i) j))
          (setf (elt tmp-t 0) 0)
          (setq torque (norm tmp-t))
          (when (or (not min-torque) (< torque min-torque))
            (setq min-torque torque)
            (setq p-idx j)))

        (if (elt (elt pose-list i) p-idx)
            (progn
              (push (elt (elt pose-list i) p-idx) pl)
              (push (elt (elt base-list i) p-idx) bl)
              (push (elt (elt torque-list i) p-idx) tl))
          (progn
            (push nil pl)
            (push nil bl)
            (push nil tl)))
      (setq p (elt (elt pose-list i) p-idx))))

    (setq motion (list (reverse pl) (reverse bl) (reverse tl)))
    motion))


(defun execute-door-motion (avs base-cds-list)
  (let ((t-interval 5000.0) ;msec
        traj-points
        diff-v
        diff-cds
        sttm)

    (send *pr2* :move-to (elt base-cds-list 0) :world)
    (dotimes (i (length base-cds-list))
      ;; (if (= i 0)
          (setq diff-cds (send (send *pr2* :copy-worldcoords) :transformation (elt base-cds-list i)))
        ;; (setq diff-cds (send (elt base-cds-list (- i 1)) :transformation (elt base-cds-list i))))
          (setq diff-v (send diff-cds :pos))
          (push (float-vector (elt diff-v 0)
                              (elt diff-v 1)
                              (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1)))
                              traj-points))

    (send *ri* :angle-vector (car avs) 1000)
    (send *ri* :wait-interpolation)
    (setq sttm (ros::time+ (ros::time-now) (ros::time 2)))
    (setq *traj-points* traj-points)
    (setq *base-cds-list* base-cds-list)
    (setq *avs* avs)

    (send *ri* :angle-vector-sequence avs (list t-interval) :default-controller sttm)
    (print "check")
    (send *ri* :move-trajectory-sequence 
          (reverse traj-points)
          (make-list (length base-cds-list) :initial-element t-interval)
          :start-time sttm
          :send-action t)
    ))

