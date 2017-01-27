(load "package://pr2eus/pr2-interface.l")
(load "pull-open-door.l")
;; (load "move-to-door-front.l")

(pr2-init)

(defun main ()
  ;; (move-to-door-grasp-knob)
  (init-door-model)
  (rotate-knob)
  (open-door)
  ;;;finished opening door
  ;; (send *ri* :move-to (send (send (send *scene* :spot "/eng2/7f/room73B2-door-front") :copy-worldcoords) :translate #f(-1000 200 0)))
  ;(speak-jp "さよなら、またね")
  )
