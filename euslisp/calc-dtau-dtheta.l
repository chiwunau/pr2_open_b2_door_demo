(setq c (make-cube 10 1200 1000))
(send c :translate #f(500 300 700))
(setq tgt (make-coords))
(send tgt :translate (send *pr2* :rarm :end-coords :worldpos))
(send tgt :translate #f(300 0 0))
(setq force-cds (make-cascoords :pos (copy-object (send tgt :worldpos))))
(send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)

(defmethod cascaded-link
  (:calc-torque-from-ext-wrenches
   (&key (force-list) (moment-list) (target-coords) ((:jacobi tmp-jacobi)))
   (let* ((link-list (mapcar #'(lambda (tc) (send self :link-list (send tc :parent))) target-coords))
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
       (if (position (send (elt ul i) :joint) joint-list)
           (setf (elt ret-tq (position (send (elt ul i) :joint) joint-list)) (elt tq i))))
     ret-tq)))


(setq link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent)))
(dotimes (i 10)
(send *pr2* :inverse-kinematics tgt
      :move-target (send *pr2* :rarm :end-coords) 
      :link-list link-list
      :stop 100
     ;;  :null-space #f(0 0 0 0.0 0.0 0.0 0 0 0 0 0.1)
;      :null-space #f(0.0 0 0 0.0 0 0 0.0 0.02)
      :null-space #'(lambda () 
                      (let ((z (calc-torque-norm-gradient #f(0 0 30) #f(0 0 0) (send tgt :copy-worldcoords)  :link-list link-list :move-target (send *pr2* :rarm :end-coords))))
;                            (print z)
                            (scale -0.0001 z)))
;      :collision-avoidance-link-pair (mapcar #'(lambda (l) (list l c)) (send *pr2* :rarm :links)) 
      :debug-view :no-message
      :rotation-axis t
      :use-torso t
;      :use-base 0.5
      )
(setq force-cds (make-cascoords :coords tgt));;:pos (copy-object (send tgt :worldpos))))
(send (send *pr2* :rarm :end-coords :parent) :assoc force-cds)
(setq res (send *pr2* :calc-torque-from-ext-wrenches :force-list (list #f(0 0 30)) :moment-list (list #f(0 0 0)) :target-coords (list force-cds)))
;(send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 5 res))
;(send *irtviewer* :viewer :viewsurface :flush)
(print (v- (send *pr2* :max-torque-vector) res))
(print (norm res)))

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

(defun calc-torque-norm-gradient (force moment wrenches-cds &key link-list move-target)
  (let* ((dtau/dq (calc-jacobian-gradient force moment wrenches-cds :link-list link-list :move-target move-target))
         tau-tmp
         (tau (instantiate float-vector (length link-list)))
         (tmp-W (instantiate float-vector (length link-list)))
         (W (unit-matrix (length link-list))) ;; coefficient tau/tau_max
         z)

    (let ((wrenches-tgt (make-cascoords :pos (copy-object (send move-target :worldpos)) :rot (copy-object (send wrenches-cds :worldrot)))))
      (send (send move-target :parent) :assoc wrenches-tgt)
      (setq tau-tmp (send *pr2* :calc-torque-from-ext-wrenches :force-list (list force) :moment-list (list moment) :target-coords (list wrenches-tgt))))
;;    (print tau-tmp)

    (let ((i 0) idx)
      (dolist (l link-list)
        (setq idx (position (send l :joint) (send *pr2* :joint-list) :test #'equal))
        (setf (elt tau i) (elt tau-tmp idx))
        (setf (elt tmp-W i) (abs (/ (elt tau-tmp idx) (elt (send *pr2* :max-torque-vector) idx))))
        (incf i)))

    (setq tmp-W (normalize-vector tmp-W)) ;;normalize W
    (dotimes (i (length tmp-W))
      (setf (aref W i i) (elt tmp-W i)))

;;    (print tau)
    ;; (print dtau/dq)
    ;; (print tau)
    ;; (print W)
    (setq z (transform W (v+ (scale 0.5 (transform (transpose dtau/dq) tau)) (scale 0.5 (transform tau dtau/dq)))))
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
      (let ((diff-jacobi-joint (calc-jacobian-joint-gradient j :move-target move-target :link-list link-list :jacobi tmp-jacobi))
            (l (position (send j :child-link) link-list :test #'equal)))
        
        (dotimes (r fik-len)
          (setf (aref fik r l) (v. (matrix-column diff-jacobi-joint r) (concatenate float-vector f-proj m-proj))))))
    fik))

(defun calc-jacobian-joint-gradient (j &key move-target link-list ((:jacobi tmp-jacobi)))
  (let ((ang (send j :joint-angle))
        (dq 0.001)
        jacobi-q jacobi-q+dq
        diff-jacobi)

    (setq jacobi-q (if tmp-jacobi tmp-jacobi
            (send *pr2* :calc-jacobian-from-link-list link-list :move-target move-target :rotation-axis t :translation-axis t)))
    (send j :joint-angle (+ ang dq))
    (setq jacobi-q+dq (send *pr2* :calc-jacobian-from-link-list link-list :move-target move-target :rotation-axis t :translation-axis t))
    (setq diff-jacobi (scale-matrix (/ 1 (deg2rad dq)) (m- jacobi-q+dq jacobi-q)))
    (send j :joint-angle ang)
    diff-jacobi))
    
  
        


    
angle :   50.0   -60.0    74.0   -70.0  -120.0   -20.0   -30.0   180.0 
 min  :   11.5  -122.3   -20.3  -214.9  -121.5    -inf  -114.6    -inf 
 max  :  325.0    32.3    74.3    37.2    -8.6     inf    -5.7     inf 
usrwei:  1.000   1.000   1.000   1.000   1.000   1.000   1.000   1.000 
addwei:  0.005   1.000   1.000   1.000   1.000   1.000   1.000   1.000 
ocost :100000000000000000000.000 100000000000000000000.000 100000000000000000000.000 100000000000000000000.000 100000000000000000000.000 100000000000000000000.000 100000000000000000000.000 100000000000000000000.000 
cost  : 51.839   0.310 1353.921   0.142 680.689   0.000   2.429   0.000 
weight:  0.005   1.000   1.000   1.000   1.000   1.000   1.000   1.000 
k     :  0.000 (manipulability:  0.174, gain:  0.001, limit:  0.100, len:8)
nspace:  0.002   0.022  -0.566  -0.013   0.542   0.000  -0.176   0.000 
J     :  0.812   0.196  -0.084  -0.048  -0.151  -0.000   0.000   0.000 
         0.512  -0.360  -0.102   0.343  -0.163   0.090  -0.000   0.000 
        -0.281  -0.090  -0.005   0.136   0.430   0.000   0.180   0.000 
         0.000   0.812   0.543  -0.840   0.171   0.866   0.000   1.000 
         0.000   0.512  -0.482  -0.296  -0.940  -0.000  -1.000  -0.000 
         0.000  -0.281   0.688   0.455  -0.296   0.500  -0.000   0.000 
d(JJt):  0.030
J#t   :  0.136   3.710  -1.304   2.526  -1.143   0.907   2.854  -0.772 
         0.035  -1.313  -1.728   0.418  -0.692   0.849   0.688   1.737 
         0.002   2.230  -0.992   2.860   2.558   1.534  -1.631  -0.635 
         0.002  -0.106  -0.142  -0.240   0.029   0.371   0.058   0.635 
        -0.015   0.414  -0.235   0.498   0.429   0.357  -1.225  -0.173 
        -0.003   0.612   0.870   0.812   0.043   0.433  -0.387  -0.670 
x     :  0.019  -0.011   0.097  -0.389  -0.116  -0.292 
J#x   :  0.237   6.777 -15.727   6.932   9.259  -8.884   6.932  -7.304 
Ny    :  0.005  -0.033  -0.025  -0.063   0.017   0.083  -0.002  -0.087 
dav   :  0.241   6.744 -15.752   6.869   9.275  -8.801   6.930  -7.392 
dav^  :  0.241   6.744 -15.752   6.869   9.275  -8.801   6.930  -7.392 
