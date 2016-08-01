(setq c (make-cube 10 1200 1000))
(send c :translate #f(500 300 700))
(setq tgt (make-coords))
(send tgt :translate (send *pr2* :rarm :end-coords :worldpos))
(send tgt :translate #f(300 0 0))
(setq force-cds (make-cascoords :pos (copy-object (send tgt :worldpos))))
(send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)


(setq link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
(dotimes (i 10)
  (unless 
      (send *pr2* :inverse-kinematics tgt
                :move-target (send *pr2* :rarm :end-coords) 
                :link-list link-list
                :stop 100
                ;;  :null-space #f(0 0 0 0.0 0.0 0.0 0 0 0 0 0.1)
                                        ;      :null-space #f(0.0 0 0 0.0 0 0 0.0 0.02)
                :null-space #'(lambda () 
                                (let ((z (calc-torque-norm-gradient #f(0 0 30) #f(0 0 0) (send tgt :copy-worldcoords) :link-list link-list :move-target (send *pr2* :rarm :end-coords) :use-base t)))
                                        ;                           (print z)
                                  (scale -0.001 z)))
                                        ;      :collision-avoidance-link-pair (mapcar #'(lambda (l) (list l c)) (send *pr2* :rarm :links)) 
                :debug-view :no-message
                :rotation-axis t
                :use-torso 0
                :use-base t
                )
    (return nil))
  (setq force-cds (make-cascoords :coords tgt));;:pos (copy-object (send tgt :worldpos))))
  (send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)
  (setq res (send *pr2* :calc-torque-from-ext-wrenches :force-list (list #f(0 0 30)) :moment-list (list #f(0 0 0)) :target-coords (list force-cds)))
                                        ;(send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 5 res))
                                        ;(send *irtviewer* :viewer :viewsurface :flush)
  (print (mapcar #'(lambda (a b) (/ a b)) (coerce res cons) (coerce (send *pr2* :max-torque-vector) cons)))
  (print (norm res))
  (send *irtviewer* :draw-objects))
      ;; :additional-check #'(lambda ()
      ;;                       (print (send *pr2* :angle-vector))
      ;;                       nil))
      :additional-check #'(lambda ()
                            (let (res)
;;                              (print (length (send *pr2* :link-list (send force-cds :parent))))
                              (setq force-cds (make-cascoords :pos (copy-object (send tgt :worldpos))))
                              (send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)
                              (setq res (send *pr2* :calc-torque-from-ext-wrenches :force-list (list #f(10 0 0)) :moment-list (list #f(0 0 0)) :target-coords (list force-cds)))
                              (send force-cds :draw-on :flush nil :size 100 :color #f(1 0 0))
;;                              (print res)
                              (setq *tg* res)
                              (setf (elt res 0) 0)
                              (send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 5 res))
                              (send *irtviewer* :viewer :viewsurface :flush)
                              (print (norm res))
                              (if (<= (norm res) 30)
                                  t
                                nil))))

      ;; :additional-check #'calc-robot-torque)

(link-list (mapcar #'(lambda (tc) (send self :link-list (send tc :parent))) target-coords))

(defun calc-robot-torque ()
  (let ((force #f(100 0 0))
        (moment #f(0 0 0))
        (target-cds (send *pr2* :rarm :end-coords))
        res)
    (setq res (send *pr2* :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords (list target-cds)))
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
    (setq force (send target-cds :inverse-transform-vector (send cds :transform-vector force))) ;;force in contact-cds(robot)
    (send *pr2* :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords (list target-cds))
    )
  )

(defun calc-torque-norm-gradient (force moment wrenches-cds &key link-list move-target (use-base nil) (base-range (list :min #f(-30 -30 -30) :max #f(30 30 30))))
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
    
