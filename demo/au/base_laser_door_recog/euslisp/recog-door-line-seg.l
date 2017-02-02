(defun start-recog ()
  (setq *door-plane-line* nil)
  (reset-scene)
  (dotimes (i 3)
    (test :max-wait-sec 0.1))
  
  (progn (setq *all-robot-pose* (reverse *all-robot-pose*)) (print t))
  (progn (setq *all-robot-base* (reverse *all-robot-base*)) (print t))
  (progn (setq *all-refined-line-segments* (reverse *all-refined-line-segments*)) (print t))

  (setq *fix-scene-line-seq* *all-refined-line-segments*)




  (setq fix-line-clusters (clustering-line-segments (car *fix-scene-line-seq*)))
  (setq refined-fix-line-segments (mapcar #'(lambda (lg) (refine-line-segments-group lg)) fix-line-clusters))
  (let ((posi (position (car (sort (send-all  refined-fix-line-segments :length) #'>)) (send-all  refined-fix-line-segments :length))))
    (setq *door-plane-line* (elt refined-fix-line-segments posi)))
  (draw-line-segments (list *door-plane-line*) :color :green)
  (send *irtviewer* :draw-objects :flush t)
  ;; (print "check")

  (do-until-key
   (setq *all-line-segments* nil)
   (setq *line-segments* nil)
   (setq *all-refined-line-segments* nil)
   (setq *all-robot-pose* nil)
   (setq *all-robot-base* nil)

   (test :max-wait-sec 0.1)
   (setq *move-line-seq* nil)
   (setq *fix-line-seq* nil)
   (let* (
          fix-line-in-new-observe
          move-line-in-new-observe)
     (dolist (new-observe *all-refined-line-segments*)
       (setq fix-line-in-new-observe nil)
       (setq move-line-in-new-observe nil)
       (dolist (nobl new-observe)
         (dolist (fix-l refined-fix-line-segments)
           (when (and (or (<= (rad2deg (abs (vector-angle (send nobl :direction) (send fix-l :direction) #f(0 0 1)))) 5)
                          (<= (rad2deg (- pi (abs (vector-angle (send nobl :direction) (send fix-l :direction) #f(0 0 1))))) 5))
                      (<= (send nobl :distance fix-l) 50)) ;;5cm
             (push nobl fix-line-in-new-observe)
             (return)))
         (unless (member nobl fix-line-in-new-observe)
           (push nobl move-line-in-new-observe)))
       (push move-line-in-new-observe *move-line-seq*)
       (push fix-line-in-new-observe *fix-line-seq*)
       )
     
     (setq *refined-fix-line-segments* refined-fix-line-segments)
     (setq *move-line-seq* (reverse *move-line-seq*))
     (setq *fix-line-seq* (reverse *fix-line-seq*))
     )

   (progn (extract_door_lines) (print t))

   (when (and *final-line-seq* (car *final-line-seq*) (< (- (elt (send (car *final-line-seq*) :point 0.5) 0) (elt (send *door-plane-line* :point 0.5) 0)) -50)  (> (send (car *final-line-seq*) :length) 50))
     ;; (when (and *final-line-seq* (car *final-line-seq*))
     (let ((ang
            (if (<= (rad2deg (abs (vector-angle (send (car *final-line-seq*) :direction) (send *door-plane-line* :direction) #f(0 0 1)))) 90)
                (rad2deg (abs (vector-angle (send (car *final-line-seq*) :direction) (send *door-plane-line* :direction) #f(0 0 1))))
              (rad2deg (- pi (abs (vector-angle (send (car *final-line-seq*) :direction) (send *door-plane-line* :direction) #f(0 0 1)))))))
           (msg (instance std_msgs::float64 :init)))
           (send msg :data ang)
           (ros::publish "/door_recog/door_angle" msg)
       (print ang))
     (clear-scene)
     (draw-line-segments (list *door-plane-line*) :color :green :fat 8)
     (draw-line-segments (remove (elt *final-line-seq* 0) (elt *all-refined-line-segments* 0)) :color :grey)
     (draw-line-segments *final-line-seq* :color :red :fat 8)
     (send *irtviewer* :draw-objects :flush t)
   )
   )
  )


(ros::advertise "/door_recog/door_angle" std_msgs::float64 1)


