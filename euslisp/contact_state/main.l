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
;;  (set-contact-points)
  (motion-generation))

(defun test-graph()
  (init)
  (load "data/motion-data-2016-10-13.dat")
  (filter-samples)
  (init-all-node :door-weight 0.0)
  (bench (init-graph)))
