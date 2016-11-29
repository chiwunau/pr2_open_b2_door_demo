(require "package://pr2eus/pr2-interface.l")

(unless (boundp '*irtviewer*)
  (make-irtviewer))
(unless (boundp '*pr2*)
  (pr2)
  (send *irtviewer* :objects *pr2*))
(load "make-door.l")
(ros::roseus-add-msgs "visualization_msgs")
(ros::roseus "door_model_fiiting")


(setq *all-cb-line-segments* nil)
(setq *line-cb-segments* nil)
(setq *stored-line-segments* nil)
(setq *line-segments-groups* nil)
(setq *all-refined-line-segments* nil)
(setq *refined-line-segments* nil)
(setq *robot-start-cds* nil)
(setq *all-robot-pose* nil)
(setq *all-robot-base* nil)
(setq *move-line-seq* nil)
(setq *fix-line-seq* nil)
(setq *final-line-seq* nil)
(setq *refined-fix-line-segments* nil)



(defun joint-state-cb (msgs)
  (apply-joint_state msgs *pr2*))

(defun line-segment-cb (msgs)
  (ros::ros-info "line segment callback")
  (unless (send msgs :markers)
    (ros::ros-info "no line segment detected")
    (return-from line-segment-cb))
  (setq *line-segments* nil)
  (let ((marker-list (send msgs :markers))
        trans)

    (setq trans (send *tfl* :lookup-transform "/odom_combined" "/base_footprint" (ros::time 0)))
    (if (not *robot-start-cds*)
        (setq *robot-start-cds* trans))
    (send *pr2* :move-to (send *robot-start-cds* :transformation trans) :world)

    (dolist (mk marker-list)
      (let (p0 p1 l)
        (setq p0 (ros::tf-point->pos (car (send mk :points))))
        (setq p1 (ros::tf-point->pos (cadr (send mk :points))))
        (setq p0 (send (send *pr2* :copy-worldcoords) :transform-vector p0))
        (setq p1 (send (send *pr2* :copy-worldcoords) :transform-vector p1))
        (setf (elt p0 2) 0)
        (setf (elt p1 2) 0)
        (setq l (make-line p0 p1))
        (push l *all-cb-line-segments*)
        (push l *cb-line-segments*)))))

(defun draw-line-segments (&optional (target *line-segments*) &key (color nil))
  (let (cylinder
        (random-color? (if color nil t)))
      (dolist (l target)
        (if random-color?
            (setq color (float-vector (random 1.0) (random 1.0) (random 1.0))))
      (setq cylinder (make-cylinder 10 (norm (v- (send l :nvertex) (send l :pvertex)))))
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

(defun generate-door-from-line-segment (tgt-line fixed-line &optional (knob-height nil) (knob-radius nil))
  (let (ln fixed-ln)
    (if (< (vector-angle #f(0 0 1) (normalize-vector (v* (matrix-column (send *pr2* :rot) 0) (send tgt-line :direction)))) pi/2)
        (progn;;cross product point up
          (print (send tgt-line :nvertex))
          (print (send tgt-line :pvertex))
          (setq ln (make-line (send tgt-line :nvertex) (send tgt-line :pvertex))))
      (setq ln line))
    (if (< (vector-angle #f(0 0 1) (normalize-vector (v* (matrix-column (send *pr2* :rot) 0) (send fixed-line :direction)))) pi/2) ;;cross product point up
        (setq fixed-ln (make-line (send fixed-line :nvertex) (send fixed-line :pvertex)))
      (setq fixed-ln fixed-line))

  ;; (if (< (norm (v- (send line-org :pvertex) (send *pr2* :pos))) (norm (v- (send line-org :nvertex) (send *pr2* :pos))))
  ;;     (setq line (make-line (send line-org :nvertex) (send line-org :pvertex)))
  ;;   (setq line line-org))
      (let* ((rotate-center (send ln :nvertex))
             (radius (send ln :length))
             (door (instance door :init
                             (- radius 95)
                             (if (numberp knob-height) knob-height 1000)
                             (if (numberp knob-radius) knob-radius 80)))
        )
        (send (car (send door :links)) :locate (send ln :pvertex))
        (let ((hinge-cds (send (car (send door :links)) :copy-worldcoords)))
          (send (car (send door :links)) :rotate (+ (/ pi 2) (vector-angle (send hinge-cds :x-axis) (send fixed-ln :direction) #f(0 0 1))) :z))
        
        (send door :hinge :joint-angle (rad2deg (vector-angle (send fixed-ln :direction) (send ln :direction) #f(0 0 1))))
        ;;    (send (car (send door :links)) :rotate
        door)))

(defun test (&key (sec 1))
  (clear-scene)
  (let ((st-tm (ros::time-now))
        accumulated-line-segments
        line-clusters
        refined-line-segments)
    (while (and (ros::ok) (ros::time<= (ros::time- (ros::time-now) st-tm) (ros::time sec)))
      (setq *cb-line-segments* nil)
      (ros::sleep)
      (ros::spin-once)
      (when *cb-line-segments*
       (draw-line-segments *cb-line-segments*)
       (setq accumulated-line-segments (append accumulated-line-segments *cb-line-segments*)))
      )
    (clear-scene)
  
  ;;;FIRST REFINEMENT
    (setq line-clusters (clustering-line-segments accumulated-line-segments))
    (dolist (line-cluster line-clusters)
      (push (refine-line-segments-group line-cluster) refined-line-segments))
    ;;SECOND REFINEMENT
    (setq line-clusters (clustering-line-segments refined-line-segments))
    (setq refined-line-segments nil)
      (dolist (line-cluster line-clusters)
        (push (refine-line-segments-group line-cluster) refined-line-segments))
      (when refined-line-segments
        (push (send *pr2* :angle-vector) *all-robot-pose*)
        (push (send *pr2* :copy-worldcoords) *all-robot-base*)
        (push refined-line-segments *all-refined-line-segments*)
        (draw-line-segments refined-line-segments))))


(defun clustering-line-segments (target &optional (ang-thres 3))
  (let (clusters tmp-list) ;;((l11 l12 l13) (l21 l22 l23) ....)
    (while target
      (setq seed (car target))
      (setq tmp-list nil)


      (dolist (ln target) ;;found lines on the same line
        (if (and (or (< (rad2deg (abs (vector-angle (send seed :direction) (send ln :direction) #f(0 0 1)))) ang-thres)
                     (< (rad2deg (abs (- pi (vector-angle (send seed :direction) (send ln :direction) #f(0 0 1))))) ang-thres))
                 (< (send seed :distance ln) 100)) ;10cm
            (push ln tmp-list)))
      
      (dolist (member tmp-list)
        (setq target (remove member target :count 1)))
      (push tmp-list clusters))
    clusters))

(defun reset-scene()
  (setq *all-line-segments* nil)
  (setq *line-segments* nil)
  (setq *all-refined-line-segments* nil)
  (setq *refined-line-segments* nil)
  (setq *all-robot-pose* nil)
  (setq *all-robot-base* nil)
)

(defun clear-scene()
  (send *irtviewer* :objects nil)
  ;; (send *irtviewer* :objects (list *pr2*))
  (send *irtviewer* :draw-objects :flush t))

(defun seperate_move_and_fix_line_segments()
  (setq *move-line-seq* nil)
  (setq *fix-line-seq* nil)
  (let*
      ((fix-line-segments (append (elt *all-refined-line-segments* 0) (elt *all-refined-line-segments* 1) (elt *all-refined-line-segments* 2)))
       (fix-line-clusters (clustering-line-segments fix-line-segments))
       (refined-fix-line-segments (mapcar #'(lambda (lg) (refine-line-segments-group lg)) fix-line-clusters))
       fix-line-in-new-observe
       move-line-in-new-observe)

    (dolist (new-observe *all-refined-line-segments*)
      (setq fix-line-in-new-observe nil)
      (setq move-line-in-new-observe nil)
      (dolist (nobl new-observe)
        (dolist (fix-l refined-fix-line-segments)
          (print (rad2deg (abs (vector-angle (send nobl :direction) (send fix-l :direction) #f(0 0 1)))))
          (print (rad2deg (- pi (abs (vector-angle (send nobl :direction) (send fix-l :direction) #f(0 0 1))))))
          (print (send nobl :distance fix-l))
          (when (and (or (<= (rad2deg (abs (vector-angle (send nobl :direction) (send fix-l :direction) #f(0 0 1)))) 4)
                         (<= (rad2deg (- pi (abs (vector-angle (send nobl :direction) (send fix-l :direction) #f(0 0 1))))) 4))
                     (<= (send nobl :distance fix-l) 10))
            (push nobl fix-line-in-new-observe)
            (return)))
        (unless (member nobl fix-line-in-new-observe)
        (push nobl move-line-in-new-observe)))
      ;; (setq fix-line-segments (append fix-line-segments fix-line-in-new-observe))
      ;; (setq fix-line-clusters (clustering-line-segments fix-line-segments))
      ;; (setq refined-fix-line-segments (mapcar #'(lambda (lg) (refine-line-segments-group lg)) fix-line-clusters))
      (push move-line-in-new-observe *move-line-seq*)
      (push fix-line-in-new-observe *fix-line-seq*)
      )
    (setq *refined-fix-line-segments* refined-fix-line-segments)
    (setq *move-line-seq* (reverse *move-line-seq*))
    (setq *fix-line-seq* (reverse *fix-line-seq*))
    ))

(defun output_detected_door_angle()
  (format t "index EF_leastsq_angle angle length~%")
          (dotimes (i (length *all-robot-pose*))
            (send *pr2* :angle-vector (elt *all-robot-pose* i))
            (send *pr2* :move-to (elt *all-robot-base* i) :world)
            (let ((arm-proj-cds (send *pr2* :larm :end-coords :copy-worldcoords)))
      (setf (elt (send arm-proj-cds :pos) 2) 0)

    (if (elt *final-line-seq* i)
        (progn 
;;          (print (v- (send arm-proj-cds :pos) circle-center))
          (draw-line-segments (list (elt *final-line-seq* i)) :color #f(1 0 0))
          (format t "~A ~A ~A ~A~%"
                  i
                  ;;nil
                  (rad2deg (abs (vector-angle (normalize-vector (v- (send arm-proj-cds :pos) circle-center)) (normalize-vector (v- (car waypoints) circle-center)) #f(0 0 1))))
;;                  (rad2deg (abs (vector-angle (send (elt *final-line-seq* i) :direction) (send (caddr *refined-fix-line-segments*) :direction) #f(0 0 1))))
                  (if (<= (rad2deg (abs (vector-angle (send (elt *final-line-seq* i) :direction) (send (cadr *refined-fix-line-segments*) :direction) #f(0 0 1)))) 90)
                      (rad2deg (abs (vector-angle (send (elt *final-line-seq* i) :direction) (send (cadr *refined-fix-line-segments*) :direction) #f(0 0 1))))
                    (rad2deg (- pi (abs (vector-angle (send (elt *final-line-seq* i) :direction) (send (cadr *refined-fix-line-segments*) :direction) #f(0 0 1))))))
                  (send (elt *final-line-seq* i) :length))
          ))
    (draw-line-segments (remove (elt *final-line-seq* i) (elt *all-refined-line-segments* i)) :color :grey)
    (unix:usleep (* 1000 200))
    (clear-scene))
            )
          )

(defun extract_door_lines ()
  (setq *final-line-seq* nil)
  (dotimes (i (length *all-robot-pose*))
    (send *pr2* :angle-vector (elt *all-robot-pose* i))
    (send *pr2* :move-to (elt *all-robot-base* i) :world)
    
    (let* ((arm-proj-cds (send *pr2* :larm :end-coords :copy-worldcoords))
           dist-list
           min-dist 
           posi
           candidate)

      (if (not (elt *move-line-seq* i))
          (push nil *final-line-seq*)
        (progn
          (setf (elt (send arm-proj-cds :pos) 2) 0)
          ;; (setq dist-list (mapcar #'(lambda (ln) (send ln :distance (send *pr2* :pos))) (elt *move-line-seq* i)))
          (setq dist-list (mapcar #'(lambda (ln) (send ln :distance (send arm-proj-cds :pos))) (elt *move-line-seq* i)))
          (setq min-dist (find-extream dist-list #'identity #'<))
          (setq posi (position min-dist dist-list))

          ;; (setq candidate (elt (elt *all-refined-line-segments* i) posi))
          (setq candidate (elt (elt *move-line-seq* i) posi))
          (print (send arm-proj-cds :pos))
          ;; (if (<= (send (elt (elt *all-refined-line-segments* i) posi) :distance (send arm-proj-cds :pos)) 100)
          (if ;;t;;(<= (send (elt (elt *move-line-seq* i) posi) :distance (send arm-proj-cds :pos)) 100)
              ;; (>= (elt (send (elt (elt *move-line-seq* i) posi) :point 0.5) 0) (elt (send *pr2* :pos) 0))
              (and (> (elt (send (elt (elt *move-line-seq* i) posi) :point 0.5) 0) 0)
              ;(<= (send (elt (elt *move-line-seq* i) posi) :distance (send arm-proj-cds :pos)) 100) ;;for robot open
                   (<= (send (elt (elt *move-line-seq* i) posi) :distance (send arm-proj-cds :pos)) 1500) ;;for human open
                   )
              (push candidate *final-line-seq*)
            (push nil *final-line-seq*))))))
  (setq *final-line-seq* (reverse *final-line-seq*)))


(defun calc-rotate-center-from-poses ()
  (setq waypoints nil)
  (setq wayseq nil)
  (dotimes (i (length *all-robot-pose*))
    (send *pr2* :angle-vector (elt *all-robot-pose* i))
    (send *pr2* :move-to (elt *all-robot-base* i) :world)
    (let ((p (send (send *pr2* :larm :end-coords :copy-worldcoords) :pos)))
      (setf (elt p 2) 0)
      (if (not waypoints)
          (push p waypoints)
        (progn
          (if (>= (norm (v- p (car waypoints))) 10)
              (push p waypoints))))))
  (setq waypoints (reverse waypoints))
)
  ;; (dotimes (j (length waypoints))
  ;;   (if (= j (- (length waypoints) 1))
  ;;       (return))
  ;;   (setq pa (elt waypoints j))
  ;;   (setq pb (elt waypoints (+ j 1)))
  ;;   (setq v (rotate-vector (v- pb pa) pi/2 :z))
  ;;   (push (make-line (scale 0.5 (v+ pa pb)) (v+ (scale 0.5 (v+ pa pb)) (scale -20 v))) wayseq))
  ;; (setq wayseq (reverse wayseq))

  ;; (dotimes (k (length wayseq))
  ;;   (if (= k (- (length wayseq) 1))
  ;;       (return))
  ;;   (push (send (elt wayseq k) :point (car (send (elt wayseq k) :intersection (elt wayseq (+ k 1))))) axis-p))




(ros::rate 10)
(setq *tfl* (instance ros::transform-listener :init))
(ros::subscribe "/markers_output" visualization_msgs::MarkerArray #'line-segment-cb)
(ros::subscribe "/joint_states" sensor_msgs::JointState #'joint-state-cb)


;; (setq l (make-line #f(100 20 0) #f(1000 50 0)))
;; (setq *door* (instance door :init (norm (v- (send l :nvertex) (send l :pvertex))) 1000 80))

;;------------------------HOW TO USE------------------;;
(defun sampling ()
  (do-until-key
   (test :sec 0.3))) ;;sampling data
(setq *all-robot-pose* (reverse *all-robot-pose*))
(setq *all-robot-base* (reverse *all-robot-base*))
(setq *all-refined-line-segments* (reverse *all-refined-line-segments*))
(seperate_move_and_fix_line_segments)
(extract_door_lines)
(setq *waypoints* (calc-rotate-center-from-poses))
;;(setq circle-center #f(840.23227 876.45468 0)) ;;robot_open_1
(setq circle-center #f(859.02818 816.39048 0)) ;;robot_open_2
(output_detected_door_angle)
;;-----------------------------------------------------;;



;;(draw-line-segments (list (refine-line-segments-group (elt *line-segments-group* 3))))
;;$ (while t (test :sec 0.5) (dolist (l *refined-line-segments*) (send *irtviewer* :objects (generatesend-door-from-line-segment l))) (send *irtviewer* :draw-objects) (unix:usleep (* 1000 500)))

;; (while t
;;   (test :sec 0.3))

;; (setq *all-robot-pose* (reverse *all-robot-pose*))
;; (setq *all-robot-base* (reverse *all-robot-base*))
;; (setq *all-refined-line-segments* (reverse *all-refined-line-segments*))
  ;; (let* ((dist-list (mapcar #'(lambda (ln) (send ln :distance (send *pr2* :pos))) *refined-line-segments*))
  ;;        (min-dist (find-extream dist-list #'identity #'<))
  ;;        (posi (position min-dist dist-list)))
  ;;   (send *irtviewer* :objects (generate-door-from-line-segment (elt *refined-line-segments* posi))x)
  ;;   (send *irtviewer* :draw-objects))
  ;; (unix:usleep (* 1000 500))
  

