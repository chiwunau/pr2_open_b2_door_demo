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
(load "~/ros/hydro/src/jsk-ros-pkg/euslib/rbrain/virtual_interpolator.l")
;;(load "graph-search.l")

(defun init()
  (unless (boundp '*pr2*)
    (pr2))
  (objects *pr2*)
  (make-door)
  (objects *door*)
  ;;temp fix for bug where collisiion model does not update
  (send *pr2* :make-collision-model-for-links)
  (send (car (send *pr2* :torso :parent-link)) :make-pqpmodel))
  ;;;;;;
  ;; (setq *left-samples* (instance motion-samples :init))
  ;; (setq *right-samples* (instance motion-samples :init)))


(defun reset-scene ()
  (send *pr2* :move-to (make-coords) :world)
  (send *pr2* :reset-pose)
  (send *door* :hinge :joint-angle 0)
  ;; (setq *left-samples* (instance motion-samples :init))
  ;; (setq *right-samples* (instance motion-samples :init))
  )


(defun demo ()
  (init)
  (init-all-container-nodes) ;;->*all-nodes*
  (setq *time1* (bench (robot-pose-sampling))) ;; fill robot pose to *all-nodes*
  (get-solvable-nodes) ;;if valid pose in *all-nodes* -> *solvable-nodes*
  (setq *time2* (bench (init-bridge-nodes :nodes *solvable-nodes*)))) ;;->bridge-nodes

(defun test-graph()
  (init)
  (load "data/nodes-data-2016-11-12-long-ver2.dat")
  (update-loaded-nodes) ;;put dependence in to coords
  (get-solvable-nodes)
  (init-start-nodes) ;;init grasp nodes of both arms
  (robot-pose-sampling :nodes *start-nodes*)
  (init-nodes-torque-info :door-weight 5)
  (bench (init-graph))
  (send *graph* :start-state (send *graph* :node "l-0-s-fk"))
  (clear-zero-relation-nodes *graph*)
  )
