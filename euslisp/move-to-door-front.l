(require "package://jsk_demo_common/euslisp/pr2-move.l")
(require "package://jsk_perception/euslisp/detection_interface.l")
(require "package://pr2eus/speak.l")

(defparameter *detection-topic* "/door_detect/ObjectDetection")

(defun look-around-for-checkerboard ()
  (let ((res (check-detection :type "door_checkerboard"
                                   :speak-name "ドア"
                                   :timeout 10
                                   :speak nil)))
    (when res
	(send res :copy-worldcoords))))

(defun move-to-door-grasp-knob ()
  (let (ckb-cds
        base-target-cds
        knob-cds)
    ;; (speak-jp "ドアに移動します")
    (send *pr2* :head :neck-p :joint-angle 0)
    (send *pr2* :head :neck-y :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (pr2-tuckarm-pose :larm)
    (send *ri* :move-to (send (send (send *scene* :spot "/eng2/7f/room73B2-door-front") :copy-worldcoords) :translate #f(-1000 200 0)))
    ;; (move-to-spot "/eng2/7f/room73B2-door-front"
    ;;               :clear-costmap t
    ;;               :tilt-enable nil)

    ;;adjust base location
    (dotimes (i 3)
      (setq ckb-cds (look-around-for-checkerboard))
      (if (or ckb-cds (= i 2))
	  (return)
	(progn (send *pr2* :head :neck-p :joint-angle 0)
	       (send *pr2* :head :neck-y :joint-angle (if (= i 0) 45 -45))
	       (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
	       (send *ri* :wait-interpolation))))
    (unless ckb-cds
      (print "困りました")
      (return-from move-to-door-grasp-knob))
    (print "found checkerboard for adjust baes pose!")
	  
	
    (objects ckb-cds)
    (setq base-target-cds (send ckb-cds :copy-worldcoords))
    (send base-target-cds :rotate (deg2rad 90) :x)
    (send base-target-cds :rotate (deg2rad 90) :z)
    (setf (elt (send base-target-cds :worldpos) 2) 0)
    (send base-target-cds :translate #f(-800 0 0))
    (send base-target-cds :translate #f(0 100 0))
    (setq knob-cds (send ckb-cds :copy-worldcoords))
    (send knob-cds :rotate (deg2rad 90) :x)
    (send knob-cds :rotate (deg2rad 90) :z)
    (send knob-cds :translate #f(-55 0 -400))
    (send knob-cds :rotate (deg2rad -90) :x)
    (objects base-target-cds)
    (objects knob-cds)
    ;;grasp knob prepare pose
    (let ((diff-cds (send (send *pr2* :copy-worldcoords) :transformation base-target-cds)))
      (send *pr2* :move-to base-target-cds :world)
      (send *pr2* :inverse-kinematics (send (send knob-cds :copy-worldcoords) :translate #f(-80 0 0))
	    :move-target (send *pr2* :larm :end-coords)
	    :link-list (send *pr2* :link-list (send *pr2* :larm :end-coords :parent))
	    :use-torso 0.001
	    )
      (send *pr2* :move-to (make-coords) :world)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
      (send *ri* :go-pos-unsafe
            (/ (elt (send diff-cds :pos) 0) 1000)
            (/ (elt (send diff-cds :pos) 1) 1000)
            (rad2deg (vector-angle #f(1 0 0) (matrix-column (send diff-cds :rot) 0) #f(0 0 1)))))
    (send *ri* :wait-interpolation)
    (send *pr2* :head :neck-p :joint-angle 0)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 500)

    ;;confirm knob coordinates
    (send *ri* :stop-grasp :larm)

    (dotimes (i 3)
      (setq ckb-cds (look-around-for-checkerboard))
      (if (or ckb-cds (= i 2))
	  (return)
	(progn (send *pr2* :head :neck-p :joint-angle (if (= i 0) 10 20))
	       (send *pr2* :head :neck-y :joint-angle 0)
	       (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
	       (send *ri* :wait-interpolation))))

      (if ckb-cds
	  (progn
	  ;;(send ckb-cds :move-to res :world)
	  (send knob-cds :move-to (send ckb-cds :copy-worldcoords) :world)
	  (send knob-cds :rotate (deg2rad 90) :x)
	  (send knob-cds :rotate (deg2rad 90) :z)
	  (send knob-cds :translate #f(-55 0 -400))
	  (send knob-cds :rotate (deg2rad -90) :x))
	(print "cannot locate checkerboard"))
    (send *irtviewer* :draw-objects :flush t)
    (send *ri* :wait-interpolation)
    ;;(unix::usleep  (* 1000 1000)) ;;ensure gripper open
    (send *pr2* :larm :inverse-kinematics (send (send knob-cds :copy-worldcoords) :translate #f(40 0 0)) :use-torso 0.0 :look-at-target t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp :larm)
    (unix::usleep (* 1000 1500))
    ;; (send base-target-cds :transate)
    ))


         

