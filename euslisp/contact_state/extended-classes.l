(defmethod cascaded-link
  ;; (:calc-torque-from-ext-wrenches
  ;;  (&key (force-list) (moment-list) (target-coords) ((:jacobi tmp-jacobi)))
  ;;  (let* ((link-list (mapcar #'(lambda (tc) (send self :link-list (send tc :parent))) target-coords))
  ;;         (jacobi
  ;;          (if tmp-jacobi
  ;;              tmp-jacobi
  ;;            (send self :calc-jacobian-from-link-list
  ;;                  link-list
  ;;                  :move-target target-coords
  ;;                  :transform-coords (mapcar #'(lambda (x) (make-coords)) target-coords)
  ;;                  :rotation-axis (make-list (length target-coords) :initial-element t)
  ;;                  :translation-axis (make-list (length target-coords) :initial-element t))
  ;;            ))
  ;;         (wrenches
  ;;          (mapcar #'(lambda (f m) (concatenate float-vector f m)) force-list moment-list))
  ;;         (tq (transform (transpose jacobi)
  ;;                        (apply #'concatenate float-vector wrenches)))
  ;;         (ret-tq (instantiate float-vector (length joint-list)))
  ;;         (ul (send self :calc-union-link-list link-list)))
  ;;    ;; tq = torque vector included in link-list                                                                            
  ;;    ;; ret-tq = all torque vector in joint-list
  ;;    (dotimes (i (length ul))
  ;;      (if (position (send (elt ul i) :joint) joint-list)
  ;;          (setf (elt ret-tq (position (send (elt ul i) :joint) joint-list)) (elt tq i))))
  ;;    ret-tq))

   ;; (:calc-joint-angle-speed-gain
   ;; (union-link-list dav periodic-time)
   ;; ;; (setq *ullj* (send-all union-link-list :joint))
   ;; ;; (setq *ull* union-link-list)
   ;; (let* ((fik-len (send self :calc-target-joint-dimension union-link-list))
   ;;        (av (instantiate float-vector fik-len)) j)
   ;;   (do* ((i 0 (+ i (send j :joint-dof)))
   ;;         (l 0 (1+ l)))
   ;;       ((>= l (length union-link-list)))
   ;;     (setq j (send (elt union-link-list l) :joint))
   ;;     (dotimes (k (send j :joint-dof))
   ;;       (setf (elt av (+ i k)) (send j :calc-angle-speed-gain dav i periodic-time))))
   ;;   av))
 (:calc-torque-from-ext-wrenches-with-base
   (&key (force-list) (moment-list) (target-coords) (link-list nil) ((:jacobi tmp-jacobi)))
   (unless link-list
     (setq link-list (mapcar #'(lambda (tc) (send self :link-list (send tc :parent))) target-coords)))
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
  
  )



(defmethod pr2-robot
  ;; (:get-virtual-link-list 
  ;;  ()
  ;;  link-list-with-virtual-link)
  (:inverse-kinematics
   (target-coords &rest args &key (link-list) (move-arm)
                  (use-torso t) (move-target) (stop 300)
                  (use-base nil) (start-coords (send self :copy-worldcoords))
                  (thre (cond
                         ((atom target-coords) 10)
                         (t (make-list (length target-coords) :initial-element 10))))
                  (rthre (cond
                         ((atom target-coords) (deg2rad 5))
                         (t (make-list (length target-coords) :initial-element (deg2rad 5)))))
		  (base-range (list :min #f(-30 -30 -30)
				   :max #f( 30  30  30)))
                  (additional-weight-list)
                  &allow-other-keys)
   (let (diff-pos-rot)
     ;;
     ;; move-arm     x o x x o o x o ;; only in this function
     ;; link-list    x x o x o x o o ;; defined in upper class
     ;; move-target  x x x o x o o o ;; defined in upper class
     ;;              1 3 x x x 4 2 x
     ;;
     ;; 1:set move-arm from target-coords, set move-target from move-arm
     ;; 2;do nothing, since move-arm is used to set link-list and move-target
     ;; 3;set move-coords and link-list from move-arm
     ;; 4;set link-list from move-arm and move-target
     (unless move-arm
       (setq move-arm (send self :select-target-arm target-coords)))
     (unless move-target
       (if (consp move-arm)
	   (setq move-target (mapcar #'(lambda (arm) (send self arm :end-coords)) move-arm))
	 (setq move-target (send self move-arm :end-coords))))
     (unless link-list
       (setq link-list
             (if (consp target-coords) ;; dual arm
                 (mapcar #'(lambda (target)
                             ;; check if target's parent within limbs
                             (let ((l target) move-arm)
                               (while l
                                 (cond ((memq l (send self :larm))
                                        (setq move-arm :larm))
                                       ((memq l (send self :rarm))
                                        (setq move-arm :rarm)))
                                 (setq l (send l :parent)))
                               (send self :link-list (send target :parent)
                                     (unless use-torso (car (send self move-arm))))))
                             move-target)
               (send self :link-list (send move-target :parent)
                     (unless use-torso (car (send self move-arm)))))))

     ;; use base
     (cond
      (use-base
       (setq diff-pos-rot
             (concatenate float-vector
                          (send start-coords :difference-position self)
                          (send start-coords :difference-rotation self)))
       (send self :move-to start-coords :world)
       (let ((res (with-append-root-joint
                   (ll self link-list
                       :joint-class omniwheel-joint
                       :joint-args base-range)
                   (send (caar ll) :joint :joint-angle
                         (float-vector (elt diff-pos-rot 0)
                                       (elt diff-pos-rot 1)
                                       (rad2deg (elt diff-pos-rot 5))))
                   ;; (setq link-list-with-virtual-link ll)
                   (send self :put :link-list-with-virtual-link ll)
                   ;; (warn (format nil "link list=~A~%" ll))
                   ;; (setq link-list-with-virtual-link ll) ;;for calc-dtau-dtheta ;;au
                   (send-super* :inverse-kinematics target-coords
                                :rthre rthre
                                :thre thre
                                :stop stop
                                :additional-weight-list
                                (append
                                 (list
                                  (list (send self :torso_lift_joint :child-link)
                                        (if (numberp use-torso) use-torso 0.005))
                                  (list (car (send self :links))
                                        (if (eq use-base t) 0.1 use-base))
                                  )
                                 additional-weight-list
                                 )
                                :link-list ll ;; link-list
                                :move-target move-target
                                args))))
         ;; (warn (format nil "--------------------------~%"))
         (send self :remprop :link-list-with-virtual-link)
         res)
       )
      (t
       (send-super* :inverse-kinematics target-coords
                    :rthre rthre
                    :thre thre
                    :stop stop
                    :additional-weight-list
                    (list
                     (list (send self :torso_lift_joint :child-link)
                           (if (numberp use-torso) use-torso 0.005))
                     )
                    :link-list link-list
                    :move-target move-target
                    args))
       )))
  )
