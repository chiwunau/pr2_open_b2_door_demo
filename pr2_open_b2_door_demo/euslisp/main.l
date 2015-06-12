(load "package://pr2eus/pr2-interface.l")
(load "open-door")
(load "move-to-door-front.l")

(pr2-init)

(defun main ()
  (move-to-door-grasp-knob)
  (init-door-model)
  (rotate-knob)
  (open-door)
  (speak-jp "さよなら、またね"))
