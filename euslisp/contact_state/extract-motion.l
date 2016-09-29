
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
          (setq st-b (elt (elt pose-list 0) r)))

    (let ((p st-p)
          next-p tmp-p min-dist dist p-idx)
      (dotimes (i (length pose-list)) ;;door angle
        (setq min-dist nil)
        (dotimes (j (length (elt pose-list i)))
          (setq tmp-p (elt (elt pose-list i) j))
          (setq dist (v- p tmp-p))
          (setf (elt dist 0) 0)
          (setq dist (norm dist))
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

    

    
    
