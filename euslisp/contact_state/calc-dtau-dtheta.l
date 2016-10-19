(require "extended-classes.l")

(defun calc-torque-norm-gradient (force moment wrenches-cds &key link-list move-target (use-base nil))
  (let* (dtau/dq 
         tau-tmp
         tau 
         tmp-W 
         W 
         z
         ll)
         

    (let ((wrenches-tgt (make-cascoords :pos (copy-object (send move-target :worldpos)) :rot (copy-object (send wrenches-cds :worldrot)))))
      (send (send move-target :parent) :assoc wrenches-tgt)
      (setq tau-tmp (send *pr2* :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords (list wrenches-tgt))))


   (cond
     (use-base
      (setq ll (car *link-list-with-virtual-link*))
      (setq dtau/dq (calc-jacobian-gradient force moment wrenches-cds :link-list ll :move-target move-target))
      (setq tau (instantiate float-vector (reduce '+ (send-all (send-all *ll* :joint) :joint-dof))));(length ll)))
      (setq tmp-W (instantiate float-vector (reduce '+ (send-all (send-all *ll* :joint) :joint-dof))));;(length ll)))
      (setq W (unit-matrix (length tau))) ;; coefficient tau/tau_max
      (let ((i 0) idx)
        (dolist (l ll)
          (if (> (send l :joint :joint-dof) 1)
              (progn 
                (replace tau (instantiate float-vector (send l :joint :joint-dof)) :start1 i)
                (replace tmp-W (instantiate float-vector (send l :joint :joint-dof)) :start1 i)
                (setq i (+ i (send l :joint :joint-dof))))
            (progn
              (setq idx (position (send l :joint) (send *pr2* :joint-list) :test #'equal))
              (setf (elt tau i) (elt tau-tmp idx))
              (setf (elt tmp-W i) (abs (/ (elt tau-tmp idx) (elt (send *pr2* :max-torque-vector) idx))))
              (incf i)))
          )))

     
     (t
      (setq dtau/dq (calc-jacobian-gradient force moment wrenches-cds :link-list link-list :move-target move-target))
      (setq tau (instantiate float-vector (length link-list)))
      (setq tmp-W (instantiate float-vector (length link-list)))
      (setq W (unit-matrix (length link-list))) ;; coefficient tau/tau_max
      (let ((i 0) idx)
        (dolist (l link-list)
          (setq idx (position (send l :joint) (send *pr2* :joint-list) :test #'equal))
          (setf (elt tau i) (elt tau-tmp idx))
          (setf (elt tmp-W i) (abs (/ (elt tau-tmp idx) (elt (send *pr2* :max-torque-vector) idx))))
          (incf i)))))
     

   (setq tmp-W (normalize-vector tmp-W)) ;;normalize W
   (dotimes (i (length tmp-W))
     (setf (aref W i i) (elt tmp-W i)))

    ;; (print "tau-tmp:")
    ;; (print tau-tmp)
    ;; (print tmp-W)
    ;; (print tau)
    ;;  (print dtau/dq)
    ;; (print W)
   (setq z (transform W (v+ (scale 0.5 (transform (transpose dtau/dq) tau)) (scale 0.5 (transform tau dtau/dq)))))
    ;; (print z)
    z))
    

    

(defun calc-jacobian-gradient (force moment wrenches-cds &key link-list move-target (rotation-axis t) (translation-axis t) ((:jacobi tmp-jacobi)))
  (let* ((fik-len (send *pr2* :calc-target-joint-dimension link-list))
;;        (dim (send *pr2* :calc-target-axis-dimension rotation-axis translation-axis))
        (fik (setq fik (make-matrix fik-len fik-len)))
        (f-proj (send move-target :inverse-transform-vector (send wrenches-cds :transform-vector force)))
        (m-proj (send move-target :inverse-transform-vector (send wrenches-cds :transform-vector moment)))
        (tmp-jacobi (if tmp-jacobi tmp-jacobi (send *pr2* :calc-jacobian-from-link-list link-list :move-target move-target :rotation-axis t :translation-axis t)))
        )
;    (print f-proj)
    (dolist (j (send-all link-list :joint))
      (dotimes (dof-count (send j :joint-dof))
        (let ((diff-jacobi-joint (calc-jacobian-joint-gradient j dof-count :move-target move-target :link-list link-list :jacobi tmp-jacobi))
              (l (position j (send-all link-list :joint) :test #'equal)))

          (setq *ll* link-list)
;;          (print diff-jacobi-joint)
          (dotimes (r fik-len)
            (setf (aref fik r (+ l dof-count)) (v. (matrix-column diff-jacobi-joint r) (concatenate float-vector f-proj m-proj)))))))
;;    (print fik)
      fik))

(defun calc-jacobian-joint-gradient (j dof-count &key move-target link-list ((:jacobi tmp-jacobi)))
  (let* ((ang (send j :joint-angle))
         (dq  0.001)
         dqv ;;for omniwheel-joint (dof > 1)
         jacobi-q jacobi-q+dq
         diff-jacobi)

    (setq jacobi-q (if tmp-jacobi tmp-jacobi
            (send *pr2* :calc-jacobian-from-link-list link-list :move-target move-target :rotation-axis t :translation-axis t)))

    (send j :joint-angle 
          (if (> (send j :joint-dof) 1)
              (progn
                (setq dqv (instantiate float-vector (send j :joint-dof)))
                (setf (elt dqv dof-count) (rad2deg dq))
                (v+ ang dqv))
            (+ ang dq)))

    (setq jacobi-q+dq (send *pr2* :calc-jacobian-from-link-list link-list :move-target move-target :rotation-axis t :translation-axis t))
    (setq diff-jacobi (scale-matrix (/ 1 dq) (m- jacobi-q+dq jacobi-q)))
    (send j :joint-angle ang)
    diff-jacobi))
    
