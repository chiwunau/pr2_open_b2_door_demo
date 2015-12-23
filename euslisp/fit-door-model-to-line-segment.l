(load "make-door.l")
(ros::roseus-add-msgs "visualization_msgs")
(ros::roseus "door_model_fiiting")


(setq *all-line-segments* nil)
(setq *line-segments* nil)
(setq *stored-line-segments* nil)
(setq *line-segments-groups* nil)
(setq *all-refined-line-segments* nil)
(setq *refined-line-segments* nil)


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

(defun draw-line-segments (&optional (target *line-segments*))
  (let (cylinder color)
    (dolist (l target)
      (setq color (float-vector (random 1.0) (random 1.0) (random 1.0)))
      (setq cylinder (make-cylinder 20 (norm (v- (send l :nvertex) (send l :pvertex)))))
      (send cylinder :translate (send l :pvertex))
      (send cylinder :rotate (deg2rad 90) :x)
      (send cylinder :rotate (+ (/ pi 2) (vector-angle #f(1 0 0) (send l :direction) #f(0 0 1))) :y)
      (send cylinder :set-color color)
      (send *irtviewer* :objects cylinder))
    (send *irtviewer* :draw-objects)))

(defun draw-line-segments-groups (&optional (target *line-segments-groups*))
  (let (cylinder color)
    (dolist (group target)
      (setq color (float-vector (random 1.0) (random 1.0) (random 1.0)))
      (dolist (ln group)
        (setq cylinder (make-cylinder 20 (norm (v- (send ln :nvertex) (send ln :pvertex)))))
        (send cylinder :translate (send ln :pvertex))
        (send cylinder :rotate (deg2rad 90) :x)
        (send cylinder :rotate (+ (/ pi 2) (vector-angle #f(1 0 0) (send ln :direction) #f(0 0 1))) :y)
        (send cylinder :set-color color)
        (send *irtviewer* :objects cylinder)))
    (send *irtviewer* :draw-objects)))

(defun refine-line-segments-group (target)
  (unless (> (length target) 1)
    (return-from refine-line-segments-group (car target)))
  (let (clusters all-vertices tmp-list seed p1 p2)
    (dolist (ln target)
      (setq all-vertices (append all-vertices (send ln :vertices))))
    (while all-vertices
      (setq tmp-list nil)
      (setq seed (car all-vertices))
      (dolist (vertex all-vertices)
        (if (< (norm (v- seed vertex)) 100)
            (push vertex tmp-list)))
      (dolist (member tmp-list)
        (setq all-vertices (remove member all-vertices)))
      (push tmp-list clusters))

    (setq *c* clusters)

    (let* ((length-list (mapcar #'length clusters))
           pos1 pos2 max1 max2 tmp)
      (setq max1 (find-extream length-list #'identity #'>))
      (setq pos1 (position max1 length-list))
      (setq tmp (remove max1 length-list :count 1))
      (setq max2 (find-extream tmp #'identity #'>))
      (setq pos2 (position max2 length-list :count (if (not (= max1 max2)) 1 2)))

      (let ((sum #f(0 0 0)))
        (dolist (p (elt clusters pos1))
          (setq sum (v+ sum p)))
        (setq p1 (scale (/ 1.0 (length (elt clusters pos1))) sum)))

      (let ((sum #f(0 0 0)))
        (dolist (p (elt clusters pos2))
          (setq sum (v+ sum p)))
        (setq p2 (scale (/ 1.0 (length (elt clusters pos2))) sum))))

    (make-line p1 p2)))

(defun generate-door-from-line-segment (line-org &optional (knob-height nil) (knob-radius nil))
  (if (< (vector-angle #f(0 0 1) (v* (matrix-column (send *pr2* :rot) 0) (send line-org :direction))) (/ pi 2)) ;;cross product point up
      (setq ln (make-line (send line-org :nvertex) (send line-org :pvertex)))
    (setq ln line-org))

  ;; (if (< (norm (v- (send line-org :pvertex) (send *pr2* :pos))) (norm (v- (send line-org :nvertex) (send *pr2* :pos))))
  ;;     (setq line (make-line (send line-org :nvertex) (send line-org :pvertex)))
  ;;   (setq line line-org))
  (let ((door (instance door :init 
                      (norm (v- (send ln :nvertex) (send ln :pvertex)))
                      (if (numberp knob-height) knob-height 1000)
                      (if (numberp knob-radius) knob-radius 80)))
        )
    (send (car (send door :links)) :locate (send ln :pvertex))
    (let ((hinge-cds (send (car (send door :links)) :copy-worldcoords)))
         (send (car (send door :links)) :rotate (+ (/ pi 2) (vector-angle (send hinge-cds :x-axis) (send ln :direction) #f(0 0 1))) :z))
;;    (send (car (send door :links)) :rotate
    door))
    
  

(defun test (&key (sec 1))
  (clear)
  (setq *stored-line-segments* nil)
  (let ((st-tm (ros::time-now)))
    (while (and (ros::ok) (ros::time<= (ros::time- (ros::time-now) st-tm) (ros::time sec)))
      (ros::sleep)
      (ros::spin-once)
      (draw-line-segments)
      (when *line-segments* 
        (setq  *stored-line-segments* (append *stored-line-segments* *line-segments*)))))
  
  (clustering-line-segments *stored-line-segments*) ;;-->*line-segments-groups*
  (clear)

  (setq *refined-line-segments* nil)
  (dolist (ln-group *line-segments-groups*)
    (push (refine-line-segments-group ln-group) *refined-line-segments*))
  (setq *all-refined-line-segments* (append *all-refined-line-segments* *refined-line-segments*))
  (draw-line-segments *refined-line-segments*))
    


(defun clustering-line-segments (&optional (target *stored-line-segments*) (ang-thres 10))
  (let (clusters tmp-list) ;;((l11 l12 l13) (l21 l22 l23) ....)
    (while target
      (setq seed (car target))
      (setq tmp-list nil)


      (dolist (ln target) ;;found lines on the same line
        (if (and (< (rad2deg (vector-angle (send seed :direction) (send ln :direction))) ang-thres)
                 (< (send seed :distance ln) 100)) ;10cm
            (push ln tmp-list)))

      (setq *line-segments-groups* nil)
      (dolist (member tmp-list)
        (setq target (remove member target :count 1)))
      (push tmp-list clusters))
    (setq *line-segments-groups* clusters)))
    


(defun reset()  
  (setq *all-line-segments* nil)
  (setq *line-segments* nil)
  (setq *all-refined-line-segments* nil)
  (setq *refined-line-segments* nil))
  
(defun clear()
  (send *irtviewer* :objects (list *pr2*))
  (send *irtviewer* :draw-objects :flush t))



(ros::rate 10)
(setq *tfl* (instance ros::transform-listener :init))
(ros::subscribe "/markers_output" visualization_msgs::MarkerArray #'line-segment-cb)




;; (setq l (make-line #f(100 20 0) #f(1000 50 0)))
;; (setq *door* (instance door :init (norm (v- (send l :nvertex) (send l :pvertex))) 1000 80))

;;------------------------HOW TO USE------------------;;
;;(draw-line-segments (list (refine-line-segments-group (elt *line-segments-group* 3))))
;;$ (while t (test :sec 0.5) (dolist (l *refined-line-segments*) (send *irtviewer* :objects (generate-door-from-line-segment l))) (send *irtviewer* :draw-objects) (unix:usleep (* 1000 500)))

(while t
  (test :sec 0.5)
  (let* ((dist-list (mapcar #'(lambda (ln) (send ln :distance (send *pr2* :pos))) *refined-line-segments*))
         (min-dist (find-extream dist-list #'identity #'<))
         (posi (position min-dist dist-list)))
    (send *irtviewer* :objects (generate-door-from-line-segment (elt *refined-line-segments* posi)))
    (send *irtviewer* :draw-objects))
  (unix:usleep (* 1000 500)))
    

