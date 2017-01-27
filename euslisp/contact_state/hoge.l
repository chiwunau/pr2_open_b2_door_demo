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


(dolist (n *solvable-nodes*)
  (if (send n :get-value :optimized)
      (play-node n :optimized t)
    (play-node n :optimized nil))
  (dotimes (i 90) ;;make sure robot's arm do not collide with rear knob
    (let ((res (pqp-collision-check-objects (send *pr2* (send n :get-value :arm)  :links) (send *door* :links))))
      (if res
          (progn (print "colli")
                 (return))
        ))

    (send *door* :hinge :joint-angle (+ i -90))
;;    (send *irtviewer* :draw-objects)
  ))

(let ((i 0))
  (dolist (n *solvable-nodes*)
    (if (send n :get-value :optimized)
        (play-node n :optimized t)
      (play-node n :optimized nil))
    (let ((res (pqp-collision-check-objects 
                      (append (list (send *pr2* :base_footprint_lk)) (send *pr2* :link-list (send (send n :get-value :contact-point) :parent)))
                      (if (eq (send n :get-value :contact-type) 'grasp)
                          (remove-if #'(lambda (x) (substringp "knob" (string (send x :name)))) (send *door* :links))
                        (send *door* :links)))))
      (if res
          (progn (print "colli")
                 (print i)
                 (return))))
    (incf i)
    (unix:usleep (* 1000 50)))
    ;;    (send *irtviewer* :draw-objects)
    ))

(let ((i 0))
  (dolist (n *bridge-nodes*)
    (if (send n :get-value :optimized)
        (play-node n :optimized t)
      (play-node n :optimized nil))
    (incf i)
       (send *irtviewer* :draw-objects)
    ))

(let ((i 0))
  (dolist (n *bridge-nodes*)
    ;; (if (send n :get-value :optimized)
    ;;     (play-node n :optimized t)
    ;;   (play-node n :optimized nil))
    (if (and (eq (send (get-node (car (send n :get-value :islands))) :get-value :contact-point-index) 0)
             ;; (eq (send (get-node (cadr (send n :get-value :islands))) :get-value :contact-point-index) 2)
             (eq (send (get-node (cadr (send n :get-value :islands))) :get-value :contact-type) 'grasp)
             )
        (progn
          (push n hoge2)
          ;; (return)
          ))
    (incf i)
    ;;    (send *irtviewer* :draw-objects)
    ))

  
(dolist (n *bridge-nodes*)
  (if (or (equal (car (send n :get-value :islands)) "r-0-2-16")
          (equal (cadr (send n :get-value :islands)) "r-0-2-16"))
      (print n)))




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




(let* ((f (open (format nil "2016-11-13/norm-ratio.txt")
                :direction :output
                :if-exists :new-version
                :if-does-not-exist :create)))

  (dolist (n *solvable-nodes*)
    (unless (send n :get-value :islands)
      (let ((tau (calc-robot-torque n :optimized nil :door-weight 5))
            (opt-tau (if (send n :get-value :robot-pose-optimized) (calc-robot-torque n :optimized t :door-weight 5) nil)))
        (when opt-tau
          (format f "~A~%" (/ (norm opt-tau) (norm tau)))))))
  (close f)
  )
;;--------------------------------------plot optimized torque comparision--------------------------------------------;;
(dolist (n *solvable-nodes*)
  ;; (print n)
  (unless (send n :get-value :islands)
    (let ((tau (calc-robot-torque n :optimized nil :door-weight 5))
          (opt-tau (if (send n :get-value :robot-pose-optimized) (calc-robot-torque n :optimized t :door-weight 5) nil))
          (min-ratio 1)
          )

      (let* ((name "norm")
             (new1? (not (probe-file (format nil "dtau_com/_~A_~A_org.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name))))
             (new2? (not (probe-file (format nil "dtau_com/_~A_~A_opt.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name))))
             (new3? (not (probe-file (format nil "dtau_com/_~A_norm_ratio.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r)))))
                    
             (f1 (open (format nil "dtau_com/_~A_~A_org.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name)
                       :direction :output
                       :if-exists :append
                       :if-does-not-exist :create))
             (f2 (open (format nil "dtau_com/_~A_~A_opt.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r) name)
                       :direction :output
                       :if-exists :append
                       :if-does-not-exist :create))
             (f3 (open (format nil "dtau_com/_~A_norm_ratio.txt" (if (eq (send n :get-value :arm) :larm) 'l 'r))
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
               (format f2 "~A~%" (norm opt-tau))
               (format f3 "~A~%" (/ (norm opt-tau) (norm tau)))

               (if (and (eq (send n :get-value :arm) :rarm) (eq (send n :get-value :contact-point-index) 0))
                   (if (< (/ (norm opt-tau) (norm tau)) min-ratio)
                       (progn (setq min-ratio (/ (norm opt-tau) (norm tau)))
                              (setq min-idx (position n *solvable-nodes*)))))
               )

             (close f1)
             (close f2)
             (close f3)
             )
      
      (dotimes (i 7)
        (let* ((name (send (elt (send *pr2* :joint-list) (if (equal :larm (send n :get-value :arm)) (+ 1 i) (+ 8 i))) :name))
               (new1? (not (probe-file (format nil "dtau_com/~A_~A_org.txt" i name))))
               (new2? (not (probe-file (format nil "dtau_com/~A_~A_opt.txt" i name))))
               (f1 (open (format nil "dtau_com/~A_~A_org.txt" i name)
                        :direction :output
                        :if-exists :append
                        :if-does-not-exist :create))
               (f2 (open (format nil "dtau_com/~A_~A_opt.txt" i name)
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

(dump-loadable-structure "data/nodes-data-dtau_com.dat" *all-nodes* *bridge-nodes*)




(defun draw-arm (&optional (arm :larm))
  (send *irtviewer* :objects (send *pr2* arm :links))
  (dolist (l (send *pr2* arm :links))
    (send *irtviewer* :objects (append (send *irtviewer* :objects) (send *pr2* :get-redundant-links l)))))

(dotimes (i (length cs))
  (setq c (make-sphere 20))
  (send c :set-color :black)
  (send c :translate (send (elt cs i) :worldpos))
  (objects c))



;;----------------------------------test obstacle------------------------------------;;
(dolist (n *all-nodes*)
  (when (eq (send n :get-value :contact-point-index) 1)
    (print n)
    (robot-pose-sampling :nodes (list n))
    ))

(get-solvable-nodes)

(dolist (n *all-nodes*)
  (unless (member n *solvable-nodes*)
    (print n)
    (robot-pose-sampling :nodes (list n))
    ))


(init-obstacle)
(send *irtviewer* :objects (append (send *irtviewer* :objects) *obs-list*))
(setq *path-list* (make-list (length *obs-list*)))
(setq *path-valid* (make-list (length *obs-list*)))

(do ((i 0 (+ i 1))) ((>= i (length *obs-list*)))
    (setq *obs* (elt *obs-list* i))
    (bench (init-graph))
    (setf (elt *path-list* i) (solve-graph *graph*))
    (if (position t (mapcar #'(lambda (x) (if (send x :get-value :solved) (colide-obstacle? x))) (send-all *path* :state)));;*path*
        (progn
          ;; (push nil *path-list*)
          (setf (elt *path-valid* i) nil)
          (send *obs* :set-color :red))
      (progn ;;(setf (elt *path-list* i) *path*)
        (setf (elt *path-valid* i) t)
        (send *obs* :set-color :green))
      )
    (send *irtviewer* :draw-objects)
    )

(dotimes (i (length *path-list*))
  (when (elt *path-list* i)
    (send (elt *obs-list* i) :set-color :yellow)
    (play-path (elt *path-list* i))
    (if (elt *path-valid* i)
        (send (elt *obs-list* i) :set-color :green)
      (send (elt *obs-list* i) :set-color :red))))


(dotimes (i (length *path-list*))
  (unless (elt *path-list* i)
    (setq *obs* (elt *obs-list* i))
    (bench (init-graph))
    (solve-graph *graph*)
    (if *path*
        (progn
          (setf (elt *path-list* i) *path*)
          (send (elt *obs-list* i) :set-color :green))
      (send (elt *obs-list* i) :set-color :red))))


;;-------------------------------------------------------------------------------------------;;
