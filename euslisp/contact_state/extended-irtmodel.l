(defmethod cascaded-link
  ;; (:calc-torque-norm-gradient
  ;;  (force moment wrenches-cds &key link-list move-target (use-base nil) (base-range (list :min #f(-30 -30 -30) :max #f(30 30 30))))
  ;;  (let* (dtau/dq 
  ;;         tau-tmp
  ;;         tau 
  ;;         tmp-W 
  ;;         W 
  ;;         z
  ;;         wrenches-tgt)

  ;;    (set wrenches-tgt (make-cascoords :pos (copy-object (send move-target :worldpos)) :rot (copy-object (send wrenches-cds :worldrot))))     (send (send move-target :parent) :assoc wrenches-tgt)
  ;;    (setq tau-tmp (send *pr2* :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords (list wrenches-tgt)))

  ;;    (cond
  ;;     (use-base

;;----------------copy from irtmodel.l---------------------------;;
 (:calc-torque-from-ext-wrenches-with-base
   (&key (force-list) (moment-list) (target-coords) (link-list (mapcar #'(lambda (tc) (send self :link-list (send tc :parent))) target-coords)) ((:jacobi tmp-jacobi)))
   (let* (
          (jacobi
           (if tmp-jacobi
               tmp-jacobi
             (send self :calc-jacobian-from-link-list
                   link-list
                   :move-target target-coords
                   :transform-coords (mapcar #'(lambda (x) (make-coords)) target-coords)
                   :rotation-axis (make-list (length target-coords) :initial-element t)
                   :translation-axis (make-list (length target-coords) :initial-element t))
             ))
          (wrenches
           (mapcar #'(lambda (f m) (concatenate float-vector f m)) force-list moment-list))
          (tq (transform (transpose jacobi)
                         (apply #'concatenate float-vector wrenches)))
          (ret-tq (instantiate float-vector (length joint-list)))
          (ul (send self :calc-union-link-list link-list)))
     ;; tq = torque vector included in link-list                                                      
     ;; ret-tq = all torque vector in joint-list                                                      
     (dotimes (i (length ul))
       (setf (elt ret-tq (position (send (elt ul i) :joint) joint-list)) (elt tq i)))
     ret-tq))

   (:calc-joint-angle-speed-gain
   (union-link-list dav periodic-time)
   ;; (setq *ullj* (send-all union-link-list :joint))
   ;; (setq *ull* union-link-list)
   (let* ((fik-len (send self :calc-target-joint-dimension union-link-list))
	  (av (instantiate float-vector fik-len)) j)
     (do* ((i 0 (+ i (send j :joint-dof)))
	   (l 0 (1+ l)))
	 ((>= l (length union-link-list)))
       (setq j (send (elt union-link-list l) :joint))
       (dotimes (k (send j :joint-dof))
	 (setf (elt av (+ i k)) (send j :calc-angle-speed-gain dav i periodic-time))))
     av)))


   
   
