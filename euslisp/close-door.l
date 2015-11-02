(load "models/room73b2-door-left-object.l")
(load "models/room73b2-door-right-object.l")

(setq *door-fix* (room73b2-door-left))
(setq *door-rotate* (room73b2-door-right))

(defun init-scene
  ()
  (send *door-fix* :translate #f(1000 405.0 0))
  (send *door-rotate* :translate #f(1000 -405.0 0)))
