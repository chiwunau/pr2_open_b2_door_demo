(defclass door
  :super cascaded-link
  :slots (end-coords l1 l2 l3 j1 j2))

(defmethod door
  (:init (door-radius knob-height knob-radius)
	 (let (b)
	   (send-super :init)
	   (setq b (make-cube 25 (* 2 knob-radius) 20))
	   (send b :locate (float-vector 0 knob-radius 0))
	   (send b :set-color :red)
	   (setq l3 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-knob))
	   (setq end-coords (make-cascoords :pos (float-vector 0 knob-radius 0)))
	   (send l3 :assoc end-coords)
	   (send l3 :locate (float-vector (+ 25 (- (+ 50 (/ 25 2.0))))  (* -1 door-radius) knob-height))
	   
	   (setq b (make-cube 50 (+ door-radius 95) (* 2 knob-height)))
	   (send b :locate (float-vector 25 (/ (+ door-radius 95) -2.0) knob-height))
	   (send b :set-color :yellow 0.8)
	   (setq l2 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-plate))
	   (send l2 :assoc l3)
	   (send l2 :locate #f(0 -25 0))
	   
	   (setq b (make-cube 50 50 (* 2 knob-height)))
	   (send b :locate (float-vector 0 0 knob-height))
	   (send b :set-color :grey)
	   (setq l1 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'door-hinge))
	   (send l1 :assoc l2)
	   
	   (setq j1 (instance rotational-joint :init :name 'j1
			      :parent-link l1 :child-link l2 :axis :z :min -100 :max 100))
	   (setq j2 (instance rotational-joint :init :name 'j1
			      :parent-link l2 :child-link l3 :axis :x :min -60 :max 0))
	   
	   (setq links (list l1 l2 l3))
	   (setq joint-list (list j1 j2))
	   (send self :init-ending)
	   self))

  (:end-coords 
   (&rest args) 
   (forward-message-to end-coords args))

  (:knob 
   (&rest args) 
   (forward-message-to (cadr (send self :joint-list)) args))
  (:hinge 
   (&rest args) 
   (forward-message-to (car (send self :joint-list)) args))
  )


