(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
;; (load "package://pr2eus_moveit/tutorials/pr2-collision-object-sample.l")
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")



(defun pub-obj (obj &optional (robot *pr2*) (link-name "base_footprint"))
  (let ((cds (send (send robot :copy-worldcoords)
                   :transformation (send obj :copy-worldcoords))))
    (send *co* :add-object obj :frame_id link-name :relative-pose cds)
    obj))


(defun test-moveit ()
  (setq *co* (instance collision-object-publisher :init))
  (send *ri* :set-moveit-environment (instance pr2-moveit-environment :init))

  (send *co* :clear-all)
  
  (dolist (obj (remove-if #'(lambda (x) (substringp "knob" (string (send x :name)))) (send *door* :links)))
    (pub-obj (car (send obj :bodies)) *pr2* "base_footprint"))
  
  (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm :arms :use-torso t))
