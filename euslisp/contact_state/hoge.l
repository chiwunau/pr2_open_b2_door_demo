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


