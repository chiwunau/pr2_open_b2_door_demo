(load "make-door.l")
(ros::roseus-add-msgs "visualization_msgs")
(ros::roseus "door_model_fiiting")


(setq *all-line-segments* nil)
(setq *line-segments* nil)


(defun line-segment-cb (msgs)
  (ros::ros-info "line segment callback")
  (unless (send msgs :markers)
    (ros::ros-info "no line segment detected")
    (return-from line-segment-cb))
  (setq *line-segments* nil)
  (let ((marker-list (send msgs :markers)))
    (dolist (mk marker-list)
      (let (p0 p1 l)
        (setq p0 (ros::tf-point->pos (car (send mk :points))))
        (setq p1 (ros::tf-point->pos (cadr (send mk :points))))
        (setq p0 (send (send *pr2* :copy-worldcoords) :transform-vector p0))
        (setq p1 (send (send *pr2* :copy-worldcoords) :transform-vector p1))
        (setf (elt p0 2) 0)
        (setf (elt p1 2) 0)
        (setq l (make-line p0 p1))
        (push l *all-line-segments*)
        (push l *line-segments*)))))

(defun draw-line-segments (&key (draw-all nil))
  (dolist (l (if draw-all *all-line-segments* *line-segments*))
    (let (c)
      (setq c (make-cylinder 10 (norm (v- (send l :nvertex) (send l :pvertex)))))
      (send c :translate (send l :pvertex))
      (send c :rotate (deg2rad 90) :x)
      (send c :rotate (+ (/ pi 2) (vector-angle #f(1 0 0) (send l :direction) #f(0 0 1))) :y)
      (send c :set-color (random-vector))
      (objects c))))

(defun generate-door-from-line-segment (line &optional (knob-height nil) (knob-radius nil))
  (let ((door (instance door :init 
                      (norm (v- (send line :nvertex) (send line :pvertex)))
                      (if (numberp knob-height) knob-height 1000)
                      (if (numberp knob-radius) knob-radius 80)))
        )
    (send (car (send door :links)) :locate (send line :pvertex))
    (let ((hinge-cds (send (car (send door :links)) :copy-worldcoords)))
         (send (car (send door :links)) :rotate (+ (/ pi 2) (vector-angle (send hinge-cds :x-axis) (send line :direction) #f(0 0 1))) :z))
;;    (send (car (send door :links)) :rotate
    door))
    
  

(defun test()
  (while (ros::ok)
    (ros::sleep)
    (ros::spin-once)
    (draw-line-segments)))

(ros::rate 5)
(setq *tfl* (instance ros::transform-listener :init))
(ros::subscribe "/markers_output" visualization_msgs::MarkerArray #'line-segment-cb)




;; (setq l (make-line #f(100 20 0) #f(1000 50 0)))
;; (setq *door* (instance door :init (norm (v- (send l :nvertex) (send l :pvertex))) 1000 80))
