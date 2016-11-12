(#<node #Xf704880 start> #<node #X104f45f8 r-1-0-1> #<node #X10e05300 r-1-1-1> #<node #Xd15de28 r-1-2-0> #<node #Xf898e70 r-1-3-1> #<node #Xfa0bcf8 r-1-4-0> #<node #Xf7046d0 end>)

(#<node #X1072c8f8 start> #<node #Xcdc3410 r-1-0-1> #<node #Xf7c0eb0 r-1-1-2> #<node #Xf982378 r-1-2-1> #<node #Xfba2fa0 r-1-3-1> #<node #X1128e668 r-1-4-0> #<node #X94e5be8 end>)



(let ((i 0))
  (while
      (not (pqp-collision-check-objects (append (send *pr2* :torso :parent-link) (send *pr2* :rarm  :links)) (send *door* :links)))
  (send *door* :hinge :joint-angle (+ i -90))
  (send *irtviewer* :draw-objects)
  (incf i))
)

(setq tmpl nil)
(dolist (dl (send *door* :links)) (unless (string-equal (send dl :name) "wall") (push dl tmpl)))



(send *irtviewer* :draw-objects)
(send *irtviewer* :viewer :viewsurface :line-width 4)
(send *irtviewer* :viewer :viewsurface :color #f(1 0 0))
(send *irtviewer* :viewer :draw-line (elt r 1) (elt r 2))
(send *irtviewer* :viewer :flush)


(dolist (n *all-node*)
  (play-node n)
  (setq r (pqp-collision-distance (car (send *pr2* :torso :parent-link)) (elt (send *door* :links) 1)))
  (send *irtviewer* :draw-objects)
  (send *irtviewer* :viewer :viewsurface :line-width 4)
  (send *irtviewer* :viewer :viewsurface :color #f(1 0 0))
  (send *irtviewer* :viewer :draw-line (elt r 1) (elt r 2))
  (send *irtviewer* :viewer :flush))


(dotimes (i 90)
  (send *door* :hinge :joint-angle (* -1 i))
  (setq r (pqp-collision-distance (car (send *pr2* :torso :parent-link)) (elt (send *door* :links) 1)))
  (send *irtviewer* :draw-objects)
  (send *irtviewer* :viewer :viewsurface :line-width 4)
  (send *irtviewer* :viewer :viewsurface :color #f(1 0 0))
  (send *irtviewer* :viewer :draw-line (elt r 1) (elt r 2))
  (send *irtviewer* :viewer :flush))

(dotimes (i 90)
  (send *door* :hinge :joint-angle (* -1 i))
  (setq r (pqp-collision-distance (elt (send *pr2* :larm :links) 1) (elt (send *door* :links) 1)))
  (send *irtviewer* :draw-objects)
  (send *irtviewer* :viewer :viewsurface :line-width 4)
  (send *irtviewer* :viewer :viewsurface :color #f(1 0 0))
  (send *irtviewer* :viewer :draw-line (elt r 1) (elt r 2))
  (send *irtviewer* :viewer :flush))


(dolist (n *all-node*)
  (play-node n)
  (dotimes (i 90) ;;make sure robot's arm do not collide with rear knob
    (let ((res (pqp-collision-check-objects (send *pr2* (send n :get :arm)  :links) (send *door* :links))))
      (if res
          (progn (print "colli")
                 (return))
        ))

    (send *door* :hinge :joint-angle (+ i -90))
;;    (send *irtviewer* :draw-objects)
  ))


122506496

(- (length *all-node*) (count t (send-all *all-node* :get :overload?)))

all-node, 787 nodes
5kg, 461 nodes
7.5kg, 162 nodes
10kg, 53 nodes

;;;;clear node in graph without parent or child
(let (childs)
  (dolist (n (send *graph* :nodes))
    (dolist (nb (send n :neighbors))
      (unless (member nb childs)
        (push nb childs))))

  (dolist (n (send *graph* :nodes))
    (when (and (not (send n :arc-list)) (not (member n childs)))
      (send *graph* :remove-node n))))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;self collision-check
(defun check-arm-self-collision
  (&optional (arm :larm))
  (let (tmpl)
    (dolist (l (send *pr2* :links)) (unless (member l (send *pr2* arm :links)) (push l tmpl)))
;    (push (car (send *pr2* :torso :parent-link)) tmpl)
  (pqp-collision-check-objects (cdr (send *pr2* arm :links)) tmpl)))



(setq buf (open "plot/torque.txt" 
                :direction :output
                :if-exists :new-version
                :if-does-not-exist :create))
(setq *torque* (reverse *torque*))

(dolist (tq *torque*)
  (format buf "~A~%" tq))

(close buf)

;;;;plot all null change torque graph;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(dotimes (i 7)
  (let* ((name (send (elt (send *pr2* :joint-list) (if (equal :larm (send n :get-value :arm)) (+ 1 i) (+ 8 i))) :name))
        (f (open (format nil "test_torque/~A_~A.txt" i name)
                      :direction :output
                      :if-exists :new-version
                      :if-does-not-exist :create)))
    (format f "~A~%" name)
    (format f "~A~%" (elt (send *pr2* :max-torque-vector) (if (equal :larm (send n :get :arm)) (+ 1 i) (+ 8 i))))
    (dolist (tq *torque*)
      (format f "~A~%" (elt tq (if (equal :larm (send n :get :arm)) (+ 1 i) (+ 8 i)))))
    
    (close f)))

(let ((f (open (format nil "test_torque/norm.txt")
              :direction :output
              :if-exists :new-version
              :if-does-not-exist :create)))
  (format f "~A~%" "Norm")
  (dolist (tq *torque*)
    (let ((tq-n (copy-object tq)))
      (setf (elt tq-n 0) 0)
      (format f "~A~%" (norm tq-n))))
      (close f))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;






(let* ((f (open (format nil "2016-11-12/norm-ratio.txt")
                :direction :output
                :if-exists :new-version
                :if-does-not-exist :create)))
  
  (dotimes (i (length tau-list))
    (if (elt opt-tau-list i)
        (format f "~A~%" (/ (norm (elt opt-tau-list i)) (norm (elt tau-list i))))
      (format f "~A~%" 0)))
  (close f)
  )
;;--------------------------------------plot optimized torque comparision--------------------------------------------;;
(dolist (n *solvable-nodes*)
  (unless (send n :get-value :islands)
    (let ((tau (calc-robot-torque n :optimized nil :door-weight 5))
          (opt-tau (if (send n :get-value :robot-pose-optimized) (calc-robot-torque n :optimized t :door-weight 5) nil)))

      (let* ((name "norm")
             (new1? (not (probe-file (format nil "2016-11-12/_~A_~A_org.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name))))
             (new2? (not (probe-file (format nil "2016-11-12/_~A_~A_opt.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name))))
             (f1 (open (format nil "2016-11-12/_~A_~A_org.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name)
                       :direction :output
                       :if-exists :append
                       :if-does-not-exist :create))
             (f2 (open (format nil "2016-11-12/_~A_~A_opt.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name)
                       :direction :output
                       :if-exists :append
                       :if-does-not-exist :create)))
             
             (when new1?
               (format f1 "~A~%" name)
               (format f1 "~A~%" 0))
             (when new2?
               (format f2 "~A~%" name)
               (format f2 "~A~%" 0))
             (when opt-tau
               (format f1 "~A~%" (norm tau))
               (format f2 "~A~%" (norm opt-tau)))

             (close f1)
             (close f2))
      
      (dotimes (i 7)
        (let* ((name (send (elt (send *pr2* :joint-list) (if (equal :larm (send n :get-value :arm)) (+ 1 i) (+ 8 i))) :name))
               (new1? (not (probe-file (format nil "2016-11-12/~A_~A_org.txt" i name))))
               (new2? (not (probe-file (format nil "2016-11-12/~A_~A_opt.txt" i name))))
               (f1 (open (format nil "2016-11-12/~A_~A_org.txt" i name)
                        :direction :output
                        :if-exists :append
                        :if-does-not-exist :create))
               (f2 (open (format nil "2016-11-12/~A_~A_opt.txt" i name)
                        :direction :output
                        :if-exists :append
                        :if-does-not-exist :create)))

          (if new1? (progn (format f1 "~A~%" name) 
                           (format f1 "~A~%" (elt (send *pr2* :max-torque-vector) (if (equal :larm (send n :get-value :arm)) (+ 1 i) (+ 8 i))))))
          (if new2? (progn (format f2 "~A~%" name)
                           (format f2 "~A~%" (elt (send *pr2* :max-torque-vector) (if (equal :larm (send n :get-value :arm)) (+ 1 i) (+ 8 i))))))

        (when opt-tau
          (format f1 "~A~%" (elt tau (if (equal :larm (send n :get-value :arm)) (+ 1 i) (+ 8 i))))
          (format f2 "~A~%" (elt opt-tau (if (equal :larm (send n :get-value :arm)) (+ 1 i) (+ 8 i)))))
          
        (close f1)
        (close f2))))))
;;---------------------------------------------------------------------------------------------------------------------------;;
