(setq c (make-cube 10 1200 1000))
(send c :translate #f(500 300 700))
(setq tgt (make-coords))
(send tgt :translate (send *pr2* :rarm :end-coords :worldpos))
(send tgt :translate #f(300 0 0))
(setq force-cds (make-cascoords :pos (copy-object (send tgt :worldpos))))
(send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)


(setq ll (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
(setq mt (send *pr2* :rarm :end-coords))
(dotimes (i 100)
      (send *pr2* :inverse-kinematics tgt
                :move-target mt
                :link-list ll
                :stop 1000
                ;;  :null-space #f(0 0 0 0.0 0.0 0.0 0 0 0 0 0.1)
                                        ;      :null-space #f(0.0 0 0 0.0 0 0 0.0 0.02)
                :null-space #'(lambda () 
;;                                (print "check")
                                (let ((z (calc-torque-norm-gradient #f(0 60 0) #f(0 0 0) (send tgt :copy-worldcoords) :link-list ll :move-target (send *pr2* :rarm :end-coords) :use-base nil)))
;;                                  (print z)
                                  (setq z (normalize-vector z))
;;                                  (print z)
                                  (scale -0.01 z)))
                :debug-view :no-message
                :rotation-axis nil
                :use-torso t
                :use-base nil
                :base-range (list :min (float-vector 0 0 -90)
                                  :max (float-vector 0 0  90))
                )


      (let ((wrenches-tgt (make-cascoords :pos (copy-object (send mt :worldpos)) :rot (copy-object (send tgt :worldrot)))))
        (send (send mt :parent) :assoc wrenches-tgt)
        (setq torque (send *pr2* :calc-torque-from-ext-wrenches :force-list (list #f(0 60 0)) :moment-list (list #f(0 0 0)) :target-coords (list wrenches-tgt))))
      ;; (send *irtviewer* :draw-objects)


                :additional-check #'(lambda ()
                                      (let* ((max-torque (send *pr2* :max-torque-vector))
                                             (overload nil)
                                             torque
                                             )
                                        (setq force-cds (make-cascoords :coords tgt));;:pos (copy-object (send tgt :worldpos))))
                                        (send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)
                                        (setq torque (send *pr2* :calc-torque-from-ext-wrenches :force-list (list #f(60 0 0)) :moment-list (list #f(0 0 0)) :target-coords (list force-cds)))
                                        (print max-torque)
                                        (print torque)
                                        (dotimes (i (length max-torque))
                                          (if (> 0 (- (abs (elt max-torque i)) (abs (elt torque i))))
                                              (setq overload t)))
                                        (print overload)
                                        (not overload)))

                )
    (return nil))
  (setq force-cds (make-cascoords :coords tgt));;:pos (copy-object (send tgt :worldpos))))
  (send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)
  (setq res (send *pr2* :calc-torque-from-ext-wrenches :force-list (list #f(50 0 0)) :moment-list (list #f(0 0 0)) :target-coords (list force-cds)))
(send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 5 res))
(send *irtviewer* :viewer :viewsurface :flush)
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


