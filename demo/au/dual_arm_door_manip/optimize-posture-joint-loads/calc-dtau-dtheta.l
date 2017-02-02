(require "extended-classes.l")

(defun calc-torque-norm-gradient (force moment wrenches-cds &key link-list move-target (use-base nil) (robot *pr2*))
  (let* (dtau/dq 
         tau-tmp
         tau 
         tmp-W 
         W 
         z
         ll)
   (let ((wrenches-tgt (make-cascoords :pos (copy-object (send move-target :worldpos)) :rot (copy-object (send wrenches-cds :worldrot)))))
     (send (send move-target :parent) :assoc wrenches-tgt)
      (if use-base
          (setq tau-tmp (send robot :calc-torque-from-ext-wrenches-with-base :force-list (list force) :moment-list (list moment) :link-list (cdr (send robot :link-list (send wrenches-tgt :parent))) :target-coords (list wrenches-tgt)))
        (setq tau-tmp (send robot :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords (list wrenches-tgt)))))

   (cond
     (use-base
      (setq ll (car (send robot :get :link-list-with-virtual-link)))
      (setq dtau/dq (calc-torque-gradient force moment wrenches-cds :link-list ll :move-target move-target :use-base use-base :robot robot))
      (setq tau (instantiate float-vector (reduce '+ (send-all (send-all ll :joint) :joint-dof))))
      (setq z (instantiate float-vector (reduce '+ (send-all (send-all ll :joint) :joint-dof))))
      (setq tmp-W (instantiate float-vector (reduce '+ (send-all (send-all ll :joint) :joint-dof))))
      (setq W (unit-matrix (length tau))) ;; coefficient tau/tau_max
      (let ((i 0) idx)
        (dolist (l ll)
          (if (> (send l :joint :joint-dof) 1)
              (progn 
                (replace tau (instantiate float-vector (send l :joint :joint-dof)) :start1 i)
                (replace tmp-W (instantiate float-vector (send l :joint :joint-dof)) :start1 i)
                (setq i (+ i (send l :joint :joint-dof))))
            (progn
              (setq idx (position (send l :joint) (send robot :joint-list) :test #'equal))
              (setf (elt tau i) (elt tau-tmp idx))
              (setf (elt tmp-W i) (abs (/ 1 (* (elt (send robot :max-torque-vector) idx) (elt (send robot :max-torque-vector) idx)))))
              (incf i)))
          )))

     (t
      (setq ll link-list)
      (setq dtau/dq (calc-torque-gradient force moment wrenches-cds :link-list ll :move-target move-target :use-base use-base :robot robot))
      (setq tau (instantiate float-vector (length ll)))
      (setq z (instantiate float-vector (length ll)))
      (setq tmp-W (instantiate float-vector (length ll)))
      (setq W (unit-matrix (length ll))) ;; coefficient tau/tau_max
      (let ((i 0) idx)
        (dolist (l ll)
          (setq idx (position (send l :joint) (send robot :joint-list) :test #'equal))
          (setf (elt tau i) (elt tau-tmp idx))
          (setf (elt tmp-W i) (abs (/ 1 (* (elt (send robot :max-torque-vector) idx) (elt (send robot :max-torque-vector) idx)))))
          (incf i)))))

   (dotimes (i (reduce '+ (send-all (send-all ll :joint) :joint-dof)))
     (let ((mr (matrix-row dtau/dq i)))
       (if use-base
           (setf (elt mr 2) (* 0.01 (elt mr 2))))
     (setq z (v+ z (scale (elt tmp-W i) (normalize-vector (scale (elt tau i) mr)))))))
   z))

(defun calc-torque-gradient (force moment wrenches-cds &key link-list move-target (rotation-axis t) (translation-axis t) (use-base nil) (jacobi nil) (robot *pr2*))
  (let* ((fik-len (send robot :calc-target-joint-dimension link-list))
         (fik (make-matrix fik-len fik-len))
         (f-proj (send (send robot :copy-worldcoords)
                       :inverse-rotate-vector force))
         (m-proj (send (send robot :copy-worldcoords)
                       :inverse-rotate-vector moment))
         (jacobi (if jacobi jacobi (send robot :calc-jacobian-from-link-list link-list :move-target move-target :transform-coords (send robot :copy-worldcoords)
                                                    :rotation-axis t :translation-axis t)))
        )
    (dolist (j (send-all link-list :joint))
      (dotimes (dof-count (send j :joint-dof))
        (let ((diff-jacobi-joint (calc-jacobian-joint-gradient j dof-count :move-target move-target :link-list link-list :jacobi jacobi :robot robot))
              (l (position j (send-all link-list :joint) :test #'equal)))

          (dotimes (r fik-len)
            (cond (use-base
                   (if (> (send j :joint-dof) 1)
                       (setf (aref fik r (+ l dof-count)) (v. (matrix-column diff-jacobi-joint r) (concatenate float-vector f-proj m-proj)))
                     (setf (aref fik r (+ l 2)) (v. (matrix-column diff-jacobi-joint r) (concatenate float-vector f-proj m-proj)))))
                  (t
                   (setf (aref fik r l) (v. (matrix-column diff-jacobi-joint r) (concatenate float-vector f-proj m-proj)))))
              ))))
      fik))

(defun calc-jacobian-joint-gradient (j dof-count &key move-target link-list (jacobi nil) (robot *pr2*))
  (let* ((ang (send j :joint-angle))
         (dq  0.1)
         (dq-om 0.1) ;;for omniwheel-joint (dof > 1)
         dqv ;;for omniwheel-joint (dof > 1)
         jacobi-q jacobi-q+dq
         base-jacobi-trans-coords
         diff-jacobi)

    (setq base-jacobi-trans-coords (send robot :copy-worldcoords))
    (setq jacobi-q (if jacobi jacobi
            (send robot :calc-jacobian-from-link-list link-list :move-target move-target :transform-coords base-jacobi-trans-coords
                  :rotation-axis t :translation-axis t)))

    (send j :joint-angle
          (if (> (send j :joint-dof) 1)
              (progn
                (setq dqv (instantiate float-vector (send j :joint-dof)))
                ;; (setf (elt dqv dof-count) (rad2deg dq))
                (setf (elt dqv dof-count) dq-om)
                (v+ ang dqv))
            (+ ang dq)))

    (setq jacobi-q+dq (send robot :calc-jacobian-from-link-list link-list :move-target move-target :transform-coords base-jacobi-trans-coords
                            :rotation-axis t :translation-axis t))
    (if (> (send j :joint-dof) 1)
        (setq diff-jacobi (scale-matrix (/ 1 dq-om) (m- jacobi-q+dq jacobi-q)))
      (setq diff-jacobi (scale-matrix (/ 1 dq) (m- jacobi-q+dq jacobi-q)))
      )
    (send j :joint-angle ang)
    diff-jacobi))
