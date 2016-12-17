(unless (boundp '*pr2*)
  (load "package://pr2eus/pr2-interface.l"))
(load "make-door.l")
;;(load "search-motion.l")
(load "generate-contact-nodes.l")
(load "generate-transition-graph.l")
(load "extract-motion.l")
(load "extended-classes.l")
(load "extended-irtmodel.l")
(load "extended-graph.l")
(load "calc-dtau-dtheta.l")
(load "write-load-data.l")
;; (load "~/ros/hydro/src/jsk-ros-pkg/euslib/rbrain/virtual_interpolator.l")
;;(load "graph-search.l")

(setq *all-nodes* nil)
(setq *bridge-nodes* nil)
(setq *start-nodes* nil) ;;temporary
(setq *solvable-nodes* nil)
(setq *door-rot-step* 20)
(setq *graph-depth* 5)
(setq *ik-target-shift* #f(50 0 0)) ;;;target coords of IK shift from door plane


(defun init()
  (unless (boundp '*pr2*)
    (pr2))
 ;; (setq *pr2* (instance extended-pr2-sensor-robot :init)))
  (objects *pr2*)
  (setq *door* (make-door))
  (objects *door*)
  (send *irtviewer* :objects (append ( send *irtviewer* :objects) (send *door* :get-rear-touch-cds-list)))
  ;;temp fix for bug where collisiion model does not update
  (send *pr2* :make-collision-model-for-links)
  (send (car (send *pr2* :torso :parent-link)) :make-pqpmodel)
  )
  ;;;;;;
  ;; (setq *left-samples* (instance motion-samples :init))
  ;; (setq *right-samples* (instance motion-samples :init)))


(defun reset-scene ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :reset-pose)
  (send *door* :hinge :joint-angle 0)
  (send *irtviewer* :objects (list *pr2*))
  (send *irtviewer* :objects *door*)
  (send *irtviewer* :objects (append ( send *irtviewer* :objects) (send *door* :get-rear-touch-cds-list)))
  (send *irtviewer* :objects (send *door* :front-knob-coords))
  (send *irtviewer* :objects (send *door* :rear-knob-coords))
        

  ;; (setq *left-samples* (instance motion-samples :init))
  ;; (setq *right-samples* (instance motion-samples :init))
  )


(defun demo ()
  (init)
  (init-all-container-nodes) ;;->*all-nodes*
  (bench (robot-pose-sampling)) ;; fill robot pose to *all-nodes*
  (get-solvable-nodes) ;;if valid pose in *all-nodes* -> *solvable-nodes*
  (bench (init-bridge-nodes :nodes *solvable-nodes*))) ;;->bridge-nodes

(defun test-graph()
  (init)
  ;; (load "data/nodes-data-2016-11-12-long-ver2.dat")
  (load "data/nodes-data-2016-12-13-new.dat")
  (update-loaded-nodes) ;;put dependence in to coords
  (get-solvable-nodes)
  (init-start-nodes) ;;init grasp nodes of both arms
  (robot-pose-sampling :nodes *start-nodes*)
  (setq *obs* nil)
  (init-nodes-torque-info :door-weight 5)
  (bench (init-graph))
  (send *graph* :start-state (send *graph* :node "l-0-s-fk"))
  (clear-zero-relation-nodes *graph*)
  )


;; (dolist (n *all-nodes*)
;;   (when (eq (send n :get-value :contact-point-index) 1)
;;     (print n)
;;     (robot-pose-sampling :nodes (list n))
;;     ))

;; (get-solvable-nodes)

;; (dolist (n *all-nodes*)
;;   (unless (member n *solvable-nodes*)
;;     (print n)
;;     (robot-pose-sampling :nodes (list n))
;;     ))


(init-obstacle)
(send *irtviewer* :objects (append (send *irtviewer* :objects) *obs-list*))
(setq *path-list* (make-list (length *obs-list*)))

(do ((i 25 (+ i 1)))  ((>= i (length *obs-list*)))
    (setq *obs* (elt *obs-list* i))
    (bench (init-graph))
    (if (send *graph* :node "l-0-s-fk")
        (progn
          (send *graph* :start-state (send *graph* :node "l-0-s-fk"))
          (solve-graph)
          (if *path*
            (progn (setf (elt *path-list* i) *path*)
                   (send *obs* :set-color :green))
            (progn
              ;; (push nil *path-list*)
              (send *obs* :set-color :red))))
      (progn
        ;; (push nil *path-list*)
        (send *obs* :set-color :red)))
    (send *irtviewer* :draw-objects))



(dotimes (i (length *path-list*))
  (when (elt *path-list* i)
    (send (elt *obs-list* i) :set-color :yellow)
    (play-path (elt *path-list* i))
    (send (elt *obs-list* i) :set-color :green)))


(dotimes (i (length *path-list*))
  (unless (elt *path-list* i)
    (setq *obs* (elt *obs-list* i))
    (bench (init-graph))
    (send *graph* :start-state (send *graph* :node "r-0-s-fk"))
    (solve-graph)
    (if *path*
        (progn
          (setf (elt *path-list* i) *path*)
          (send (elt *obs-list* i) :set-color :green))
      (send (elt *obs-list* i) :set-color :red))))

  
