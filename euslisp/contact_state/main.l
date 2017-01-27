(unless (boundp '*pr2*)
  (load "package://pr2eus/pr2-interface.l"))
(load "make-73b2-door.l")
(load "make-602exp-door.l")
(load "generate-contact-nodes.l")
(load "generate-transition-graph.l")
(load "extract-motion.l")
(load "extended-classes.l")
(load "extended-graph.l")
(load "graph-rule.l")
(load "optimize-posture-joint-loads/calc-dtau-dtheta.l")
(load "~/ros/hydro/src/jsk-ros-pkg/euslib/rbrain/virtual_interpolator.l")

(setq *all-nodes* nil)
(setq *bridge-nodes* nil)
(setq *start-nodes* nil) ;;temporary
(setq *solvable-nodes* nil)
(setq *door-rot-step* 20)
(setq *graph-depth* 6) ;;0 20 40 60 80 100
(setq *ik-target-shift* #f(50 0 0)) ;;;target coords of IK shift from door plane


(defun init()
  (unless (boundp '*pr2*)
    (pr2))
  (objects *pr2*)
  (setq *door* (make-602exp-door))
  (objects *door*)
  (send *irtviewer* :objects (append ( send *irtviewer* :objects) (send *door* :get-rear-touch-cds-list)))
  ;;temp fix for bug where collisiion model does not update
  (send *pr2* :make-collision-model-for-links)
  (send (car (send *pr2* :torso :parent-link)) :make-pqpmodel)
  )

(defun reset-scene ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :reset-pose)
  (send *door* :hinge :joint-angle 0)
  (send *irtviewer* :objects (list *pr2*))
  (send *irtviewer* :objects *door*)
  (send *irtviewer* :objects (append ( send *irtviewer* :objects) (send *door* :get-rear-touch-cds-list)))
  (send *irtviewer* :objects (send *door* :front-knob-coords))
  (send *irtviewer* :objects (send *door* :rear-knob-coords))
        
  )


(defun sampling-all-contact-states ()
  (init)
  (init-all-container-nodes) ;;->*all-nodes*
  (bench (robot-pose-sampling)) ;; fill robot pose to *all-nodes*
  (get-solvable-nodes) ;;if valid pose in *all-nodes* -> *solvable-nodes*
  (bench (init-bridge-nodes :nodes *solvable-nodes*))) ;;->bridge-nodes


;;load sampled contact states data and construct a graph
(defun test-graph()
  (init)
  (load "data/nodes-data-2016-16-12-23-602expdoor.dat")
  (update-loaded-nodes) ;;update robot and door model info in loaded nodes
  (get-solvable-nodes)
  (setq *obs* nil)
  (init-nodes-torque-info :door-weight 5)
  (bench (init-graph))
  ;; (send *graph* :start-state (send *graph* :node "l-0-s-fk"))
  (clear-zero-relation-nodes *graph*)
  (solve-graph *graph*)
  (play-path :path *path*)
  )
