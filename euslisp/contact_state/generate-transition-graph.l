(defun init-graph (&key (nodes *solvable-nodes*) (bridge-nodes *bridge-nodes*))
  (let ((start-node (instance extended-node :init "start"))
        (goal-node (instance extended-node :init "goal"))
        (graph (instance costed-graph :init))
        (w-tq 10.0)
        (w-obs 100.0))

    
    (send-all nodes :remove-all-arcs)
    (send-all bridge-nodes :remove-all-arcs)
    
    (send graph :add-node start-node)
    (send graph :add-node goal-node)

    ;;add nodes to graph
    (let ((start-depth 0)
          (goal-depth (- *graph-depth* 1)))
      (dolist (n nodes)
        (when (and (>= (send n :get-value :graph-depth) start-depth) (<= (send n :get-value :graph-depth) goal-depth))
          (send n :put :obstacle-dist-cost (obstacle-dist-cost :node n :obstacle *obs*))
          (send n :put :torque-limit-cost (torque-limit-cost :node n))
          ;; (print (send n :get :obstacle-dist-cost))
          ;; (when (and ;;(not (colide-obstacle? n))
          ;;        (not (send n :get-value :overload?)))
            (send graph :add-node n)))
        
  ;;init single point contact nodes
    (do ((depth start-depth (+ depth 1))) ((>= depth goal-depth))
        (let ((ns1 (extract-nodes-by-depth depth :nodes nodes))
              (ns2 (extract-nodes-by-depth (+ depth 1) :nodes nodes)))
          (dolist (n1 ns1)
            (when (member n1 (send graph :nodes))
              (if (eq (send n1 :get-value :graph-depth) start-depth) ;;add start node
                  (send graph :add-arc start-node n1 1 :both nil))
              (dolist (n2 ns2)
                (when (member n2 (send graph :nodes))
                  (if (and (not (member goal-node (send n2 :neighbors))) (equal (send n2 :get-value :graph-depth) goal-depth)) ;;add end node
                      (send graph :add-arc n2 goal-node 1 :both nil))
                  (cond 
                   ((or (eq (send n1 :get-value :contact-type) 'grasp) (eq (send n2 :get-value :contact-type) 'grasp)) ;;when either node is grasp knob
                    (if (and (equal (send n1 :get-value :object-coords-index) (send n2 :get-value :object-coords-index)) (equal (send n1 :get-value :arm) (send n2 :get-value :arm)))
                        (send graph :add-arc n1 n2
                              (+ (pose-diff-cost 
                                  (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose))
                                  (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose))
                                  (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :base-coords-optimized) (send n1 :get-value :base-coords))
                                  (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :base-coords-optimized) (send n2 :get-value :base-coords)))
                                 (* w-obs (send n2 :get :obstacle-dist-cost))
                                 (* w-tq (send n2 :get :torque-limit-cost)))
                              :both nil)))
                   ((and (equal (send n1 :get-value :arm) (send n2 :get-value :arm)) (equal (send n1 :get-value :contact-point) (send n2 :get-value :contact-point)))
                    (if (transitable? n1 n2)
                        (send graph :add-arc n1 n2
                              (+ (pose-diff-cost
                                  (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose))
                                  (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose))
                                  (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :base-coords-optimized) (send n1 :get-value :base-coords))
                                  (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :base-coords-optimized) (send n2 :get-value :base-coords)))
                                 (* w-obs (send n2 :get :obstacle-dist-cost))
                                 (* w-tq (send n2 :get :torque-limit-cost)))
                              :both nil))
                    )
                   )))))))
  )
  (dolist (n *bridge-nodes*)
    (send n :put :obstacle-dist-cost (obstacle-dist-cost :node n :obstacle *obs*))
    (let ((ri (get-node (car (send n :get-value :islands))))
          (li (get-node (cadr (send n :get-value :islands)))))
      (when (and (member ri (send graph :nodes)) (member li (send graph :nodes)))
        (send graph :add-node n)
        (send graph :add-arc ri n
              (+ (pose-diff-cost
                  (if (send ri :get-value :robot-pose-optimized) (send ri :get-value :robot-pose-optimized) (send ri :get-value :robot-pose))
                  (send n :get-value :robot-pose)
                  (if (send ri :get-value :robot-pose-optimized) (send ri :get-value :base-coords-optimized) (send ri :get-value :base-coords))
                  (send n :get-value :base-coords))
                 (* w-obs (/ (+ (send ri :get :obstacle-dist-cost) (send n :get :obstacle-dist-cost)) 2))
                 (* w-tq (/ (+ (send ri :get :torque-limit-cost) (send li :get :torque-limit-cost)) 4))
                 )
              :both t)
        (send graph :add-arc li n 
              (+ (pose-diff-cost
                  (if (send li :get-value :robot-pose-optimized) (send li :get-value :robot-pose-optimized) (send li :get-value :robot-pose))
                  (send n :get-value :robot-pose)
                  (if (send li :get-value :robot-pose-optimized) (send li :get-value :base-coords-optimized) (send li :get-value :base-coords))
                  (send n :get-value :base-coords))
                 (* w-obs (/ (+ (send li :get :obstacle-dist-cost) (send n :get :obstacle-dist-cost)) 2))
                 (* w-tq (/ (+ (send ri :get :torque-limit-cost) (send li :get :torque-limit-cost)) 4))
                 )
              :both t))))

  (setq *graph* graph)
  (set-start-goal-state)
  ))

(defun solve-graph (g)
  (let (solver path)
    (defmethod a*-graph-search-solver
      (:hn (n p)
           0.0))
    (setq solver (instance a*-graph-search-solver :init g))
    (setq path (send solver :solve g :verbose nil))
    (setq *path* path)
    path)
  )

(defun extract-nodes-by-depth (depth &key (nodes *all-nodes*))
  (let (rect-nodes)
     (dolist (n nodes)
       (if (eq (send n :get-value :graph-depth) depth)
           (push n rect-nodes)))
     (reverse rect-nodes)))

(defun set-start-goal-state ()
 (send *graph* :start-state (send *graph* :node "start"))
  (send *graph* :goal-state (send *graph* :node "goal"))
)


(defun clear-zero-relation-nodes (&optional (g *graph*))
  (let (childs)
    (dolist (n (send g :nodes))
      (dolist (nb (send n :neighbors))
        (unless (member nb childs)
          (push nb childs))))
    
    (dolist (n (send g :nodes))
      (when (and (not (send n :arc-list)) (not (member n childs)))
        (send g :remove-node n))))
)

(defun play-path (&key (path *path*) (t-inv 300) (save-image nil))
  (let ((ns (send-all path :state))
        (c 0))
    (dolist (n ns)
      (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
        (if (send n :get-value :robot-pose-optimized)
            (play-node n :optimized t)
          (play-node n))
        (if save-image
            (send *irtviewer* :viewer :viewsurface :write-to-image-file (format nil "save_image/~A_~A" c (send n :name))))
        (incf c)
        (unix:usleep (* 1000 t-inv))))))

(defun test-path (&optional (p *path*))
  (let ((ns (send-all p :state)))
    (dolist (n ns)
      (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
        (test-node n)))))
  

(defun test-node (n)
  (let ((o (overload-torque? (if (send n :get-value :robot-pose-optimized) (send n :get-value :torque-optimized) (send n :get-value :torque))))
        (c (colide-obstacle? n)))
    (format t "Node:~A Overload:~A Collision:~A~%" (send n :name) o c)))

(defun execute-robot-motion-one-by-one (&optional (p *path*))
  (let ((t-interval 5000.0) ;msec
        (ns (send-all p :state))
        avs
        base-cds-list
        traj-points
        diff-v
        diff-cds
        sttm
        pre-av
        pre-base
        av
        base
        traj-pt)

    (dolist (n ns)    
      (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
        (if (send n :get-value :robot-pose-optimized)
            (progn (push (send n :get-value :robot-pose-optimized) avs)
                   (push (send n :get-value :base-coords-optimized) base-cds-list))
          (progn (push (send n :get-value :robot-pose) avs)
                 (push (send n :get-value :base-coords) base-cds-list)))))
    
    (setq avs (reverse avs))
    (setq base-cds-list (reverse base-cds-list))
    (send *pr2* :move-to (elt base-cds-list 0) :world)
    (send *pr2* :angle-vector (elt avs 0))
    (setq pre-base (send *pr2* :copy-worldcoords))
    (send *ri* :start-grasp)
    

    (do ((i 0 (+ i 1))) ((>= i (length ns)))
        (let ((n (elt ns i))
              (next-n (if (< i (- (length ns) 1)) (elt ns (+ i 1))))
              (pre-n (if (> i 0) (elt ns (- i 1)))))
          (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
            (send *ri* :start-grasp)

            ;;if n is dual contact
            (when (send n :get-value :islands)
              (let ((rn (get-node (car (send n :get-value :islands))))
                    (ln (get-node (cadr (send n :get-value :islands)))))
                (if (and (eq (send pre-n :get-value :arm) :larm) (eq (send rn :get-value :contact-type) 'grasp))
                    (send *ri* :stop-grasp :rarm))
                (if (and (eq (send pre-n :get-value :arm) :rarm) (eq (send ln :get-value :contact-type) 'grasp))
                    (send *ri* :stop-grasp :larm))

                (let ((new-arm (if (eq (send pre-n :get-value :arm) :rarm) :larm :rarm))
                      new-arm-av res pre-grasp-av)
                  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
                  (setq new-arm-av (send *pr2* new-arm :angle-vector))
                  (play-node-best n :draw-objects nil)
                  (send *pr2* new-arm :angle-vector new-arm-av)
                  (setq av (send *pr2* :angle-vector))
                  (setq base (send *pr2* :copy-worldcoords))
                  (setq diff-cds (send pre-base :transformation base))
                  (setq diff-v (send diff-cds :pos))
                  (setq traj-pt (float-vector (elt diff-v 0)
                                              (elt diff-v 1)
                                              (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1))))
                  (setq pre-base (send *pr2* :copy-worldcoords))

                  
                  ;;move already contacting arm first
                  (setq sttm (ros::time+ (ros::time-now) (ros::time 1)))
                  (let* ((diff-torso-angle (abs (- (elt (send *ri* :state :potentio-vector) 0) (elt av 0))))
                         (min-t (* 1000 (/ diff-torso-angle 10))))
                    (send *ri* :angle-vector av
                          (if (> min-t t-interval) min-t t-interval)
                          :default-controller sttm)
                    (print traj-pt)
                    (send *ri* :move-trajectory-sequence
                          (list traj-pt)
                          (list (if (> min-t t-interval) min-t t-interval))
                          :start-time sttm
                          :send-action t
                          ))

                  ;;;if new contact is grasp, solve prepare posture
                  (if (eq (send (if (eq new-arm :rarm) rn ln) :get-value :contact-type) 'grasp)
                      (progn
                        (play-node-best n :draw-objects nil)
                        (setq res (send *pr2* new-arm :inverse-kinematics (send (send *pr2* new-arm :end-coords :copy-worldcoords) :translate  #f(-100 0 0))))
                        (unless res
                          (setq res (send *pr2* new-arm :inverse-kinematics (send (send *pr2* new-arm :end-coords :copy-worldcoords) :translate #f(-50 0 0)))))
                        (send *pr2* new-arm :elbow-r :joint-angle (refine-continuous-joint-ang (send *pr2* new-arm :elbow-r :joint-angle) 360))
                        (send *pr2* new-arm :wrist-r :joint-angle (refine-continuous-joint-ang (send *pr2* new-arm :wrist-r :joint-angle) 180))
                        (when res
                          (setq pre-grasp-av res)
                          (print pre-grasp-av)
                          (send *irtviewer* :draw-objects)
                          (format t  "Grasp prepare posture, uSE MoveIt for ~A!~%" new-arm)
                          (send *co* :clear-all)
                          (dolist (obj (remove-if #'(lambda (x) (substringp nil (string (send x :name)))) (send *door* :links)))
                            (pub-obj (car (send obj :bodies)) *pr2* "base_footprint"))
                          (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm new-arm :use-torso nil))
                          (if res
                              (progn
                                (send *ri* :wait-interpolation)
                                (send *ri* :angle-vector (send *pr2* :angle-vector))
                                (send *ri* :wait-interpolation)
                                (play-node-best n :draw-objects nil)
                                (send *ri* :angle-vector (send *pr2* :angle-vector)) ;;finish execute n
                                (send *ri* :wait-interpolation))
                            (progn ;;grasp pre pose not solved
                              (play-node-best n :draw-objects nil)
                              (format t  "Grasp prepare posture not solved, grasp posture use MoveIt for ~A!~%" new-arm)
                              (send *co* :clear-all)
                              (dolist (obj (remove-if #'(lambda (x) (substringp "knob" (string (send x :name)))) (send *door* :links)))
                                (pub-obj (car (send obj :bodies)) *pr2* "base_footprint"))
                              (play-node-best n :draw-objects nil)
                              (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm new-arm :use-torso nil))
                              (unless res
                                (send *pr2* :angle-vector pre-grasp-av)
                                (send *ri* :angle-vector (send *pr2* :angle-vector))
                                (send *ri* :wait-interpolation)
                                (play-node-best n :draw-objects nil)
                                (send *ri* :angle-vector (send *pr2* :angle-vector))
                                (send *ri* :wait-interpolation)
                              )))))
                          
                    (progn ;;new contact is not grasp
                      (format t  "USE MoveIt for ~A!~%" new-arm)
                      (send *co* :clear-all)
                      (dolist (obj (remove-if #'(lambda (x) (substringp nil (string (send x :name)))) (send *door* :links)))
                        (pub-obj (car (send obj :bodies)) *pr2* "base_footprint"))
                      (play-node-best n :draw-objects nil)
                      (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm new-arm :use-torso nil))
                      )
                    )
                  )))
                  
            
            (send *ri* :wait-interpolation)
            (play-node-best n)
            (setq av (send *pr2* :angle-vector))
            (setq base (send *pr2* :copy-worldcoords))
            (setq diff-cds (send pre-base :transformation base))
            (setq diff-v (send diff-cds :pos))
            (setq traj-pt (float-vector (elt diff-v 0)
                                        (elt diff-v 1)
                                        (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1))))
            
            (setq sttm (ros::time+ (ros::time-now) (ros::time 1)))
            (let* ((diff-torso-angle (abs (- (elt (send *ri* :state :potentio-vector) 0) (elt av 0))))
                   (min-t (* 1000 (/ diff-torso-angle 10))))
              (send *ri* :angle-vector av
                    (if (> min-t t-interval) min-t t-interval)
                    :default-controller sttm)
              (print traj-pt)
              (send *ri* :move-trajectory-sequence
                    (list traj-pt)
                    (list (if (> min-t t-interval) min-t t-interval))
                    :start-time sttm
                    :send-action t))
            
          ;;------------After robot moved to n----------------------


          ;;if next-n is dual contact
          ;;prepare to add one contact
          (when (send next-n :get-value :islands)
            (let ((spare-arm (if (eq (send n :get-value :arm) :rarm) :larm :rarm))
                  spare-arm-av res)
              (format t  "USE MoveIt for ~A!~%" spare-arm)
              
              (send *co* :clear-all)
              (dotimes (i 3)
                (cond ((= i 0)
                       (play-node-best n))
                      ((= i 1)
                       (play-middle-state n next-n 1 3))
                      ((= i 2)
                       (play-node-best next-n)))

                (let ((tmp-door (copy-object *door*)))
                (dolist (obj (remove-if #'(lambda (x) (substringp nil (string (send x :name)))) (send tmp-door :links)))
                  (pub-obj (car (send obj :bodies)) *pr2* "base_footprint"))))
                
                
              (play-node-best n)
              (unless res
                (pr2-relax-pose)
                (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm spare-arm :use-torso nil))
                (unless res
                  (send *pr2* :reset-pose)
                  (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm spare-arm :use-torso nil))
                  (unless res
                    (send *pr2* :reset-manip-pose)
                    (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm spare-arm :use-torso nil))
                    ))))
            )
          (send *co* :clear-all)
                    
          (when (send n :get-value :islands)
            (let ((rn (get-node (car (send n :get-value :islands))))
                  (ln (get-node (cadr (send n :get-value :islands)))))
              
              (if (eq (send rn :get-value :contact-type) 'grasp)
                  (progn
                    (send *ri* :wait-interpolation)
                    (send *ri* :start-grasp :rarm)
                    ))              
              (if (eq (send ln :get-value :contact-type) 'grasp) 
                  (progn
                    (send *ri* :wait-interpolation)
                    (send *ri* :start-grasp :larm)
                    ))

              ;;finished moving to n
              ;;if n is dual contact, preapre to sift to single contact
              (let ((spare-arm (if (eq (send next-n :get-value :arm) :rarm) :larm :rarm))
                    spare-arm-av res)
                ;;stop grasp if necessary
                (cond ((and (not (eq (send pre-n :get-value :contact-type) 'grasp)) (or (eq (send rn :get-value :contact-type) 'grasp) (eq (send rn :get-value :contact-type) 'grasp))) ;;pre not grasp -> either grasp
                       (unix:usleep (* 1000 1000)))
                      ((and (eq (send rn :get-value :contact-type) 'grasp) (eq (send ln :get-value :contact-type) 'grasp)) ;;pre grasp -> both grasp
                       (unix:usleep (* 1000 1000))))
                
                (cond ((or (and (or (eq (send rn :get-value :contact-type) 'grasp) (eq (send ln :get-value :contact-type) 'grasp)) (not (eq (send next-n :get-value :contact-type) 'grasp)));;either grasp -> next no grasp
                           (and (eq (send rn :get-value :contact-type) 'grasp) (eq (send ln :get-value :contact-type) 'grasp) (eq (send next-n :get-value :contact-type) 'grasp))) ;;both grasp -> next single grasp
                       (send *ri* :stop-grasp spare-arm)
                       (unix:usleep (* 1000 1000))
                       (play-node-best n :draw-objects nil)
                       (unless (send *pr2* spare-arm :inverse-kinematics (send (send *pr2* spare-arm :end-coords :copy-worldcoords) :translate  #f(-100 0 0)))
                         (send *pr2* spare-arm :inverse-kinematics (send (send *pr2* spare-arm :end-coords :copy-worldcoords) :translate #f(-50 0 0))))
                       (send *pr2* spare-arm :elbow-r :joint-angle (refine-continuous-joint-ang (send *pr2* spare-arm :elbow-r :joint-angle) 360))
                       (send *pr2* spare-arm :wrist-r :joint-angle (refine-continuous-joint-ang (send *pr2* spare-arm :wrist-r :joint-angle) 180))
                       (send *irtviewer* :draw-objects)
                       (send *ri* :angle-vector (send *pr2* :angle-vector))
                       (send *ri* :wait-interpolation)
                       ))
                      
                ;;prepare to release one contact
                (format t  "USE MoveIt for ~A!~%" spare-arm)
                
                (setq res nil)
                (dotimes (i 3)
                  (cond ((= i 0)
                         (play-node-best n))
                        ((= i 1)
                         (play-middle-state n next-n 1 3))
                        ((= i 2)
                         (play-node-best next-n)))

                  (if res (return))
                  (send *co* :clear-all)
                  (dolist (obj (remove-if #'(lambda (x) (substringp nil (string (send x :name)))) (send *door* :links)))
                    (pub-obj (car (send obj :bodies)) *pr2* "base_footprint"))
                  
                  (send *pr2* spare-arm :angle-vector (get-tuckarm (if (eq spare-arm :rarm) :larm :rarm) :outside spare-arm))
                  (send *pr2* spare-arm :shoulder-r :joint-angle (if (eq spare-arm :rarm) -100 100))
                  (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm spare-arm :use-torso nil))
                  (unless res
                    (send *pr2* :reset-pose)
                    (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm spare-arm :use-torso nil))
                    (unless res
                      (send *pr2* :reset-manip-pose)
                    (setq res (send *ri* :angle-vector-motion-plan (send *pr2* :angle-vector) :move-arm spare-arm :use-torso nil))
                    ))))
              

              ))
          
          (send *ri* :wait-interpolation)
          (do-until-key 
           (print "press key")
           (unix::usleep (* 1000 1000)))

        (send *pr2* :move-to base :world)
        (setq pre-base (send *pr2* :copy-worldcoords))
        )
        ))))
        

(defun init-obstacle ()
  (let* ((door-width (elt (send (car (send (elt (send *door* :links) 1) :bodies)) :body-type) 2))
         (h-step (/ door-width 4))
         (v-step (/ door-width 4))
         cube-list)
    

    (dotimes (p 4)
      (dotimes (q 12)
        (let (c) 
          (setq c (make-cube 100 100 200))
          (send c :translate (send *door* :worldpos))
          (send c :translate #f(0 0 101)) ;;base z-axis of pr2 base-coords is zero
          (send c :translate (float-vector (- (+ door-width (/ v-step 2))) (/ h-step 2) 0))
          (send c :translate (float-vector (- (* p v-step)) (- (* 3 h-step) (* q h-step)) 0))
          (push c cube-list))))

    (dotimes (p 4)
      (dotimes (q 4)
        (let (c) 
          (setq c (make-cube 100 100 200))
          (send c :translate (send *door* :worldpos))
          (send c :translate #f(0 0 101))
          (send c :translate (float-vector (- (+ door-width (/ v-step 2))) (/ h-step 2) 0))
          (send c :translate (float-vector (* (+ 1 p) v-step) (- (+ (* 5 h-step) (* q h-step))) 0))
          (push c cube-list))))

    (dotimes (p 4)
      (dotimes (q 4)
        (let (c) 
          (setq c (make-cube 100 100 200))
          (send c :translate (send *door* :worldpos))
          (send c :translate #f(0 0 101))
          (send c :translate (float-vector (- (+ door-width (/ v-step 2))) (/ h-step 2) 0))
          (send c :translate (float-vector (* (+ 1 p) v-step) (- (+ (* -3 h-step) (* q h-step))) 0))
          (push c cube-list))))

    (setq *obs-list* (reverse cube-list))))


(defun pr2-relax-pose ()
  (send *pr2* :larm :angle-vector #f(66.0749 39.4186 172.313 -70.4624 2.60698 -5.72958 -10.1163))
  (send *pr2* :rarm :angle-vector #f(-66.0749 39.4186 -172.313 -70.4624 2.60698 -5.72958 -10.1163))
  )
        

    
