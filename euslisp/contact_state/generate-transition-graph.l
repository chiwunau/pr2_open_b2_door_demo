(defun init-graph (&key (nodes *solvable-nodes*) (bridge-nodes *bridge-nodes*))
  (let ((start-node (instance extended-node :init "start"))
        (goal-node (instance extended-node :init "goal"))
        (graph (instance costed-graph :init)))
    
    (send-all nodes :remove-all-arcs)
    (send-all bridge-nodes :remove-all-arcs)
;;     ;; (send start-node :remove-all-arcs)
;;     ;; (send goal-node :remove-all-arcs)
    
    (send graph :add-node start-node)
    (send graph :add-node goal-node)

;;   ;;add nodes to graph
  (dolist (n nodes)
    (send graph :add-node n))
            
  ;;init single point contact nodes
  (dotimes (depth (- *graph-depth* 1))
    (let ((ns1 (extract-nodes-by-depth depth :nodes nodes))
          (ns2 (extract-nodes-by-depth (+ depth 1) :nodes nodes)))
      (setq *ns1* ns1)
      ;; (print ns1)
      ;; (print ns2)
      ;; (print "--------")
      (dolist (n1 ns1)
;;        (print n1)
        (when (member n1 (send graph :nodes))
;          (print n1)
          (if (eq (send n1 :get-value :graph-depth) 0) ;;add start node
               (send graph :add-arc start-node n1 1 :both nil))
          (dolist (n2 ns2)
            (when (member n2 (send graph :nodes))
              (if (and (not (member goal-node (send n2 :neighbors))) (equal (send n2 :get-value :graph-depth) (- *graph-depth* 1))) ;;add end node
                    (send graph :add-arc n2 goal-node 1 :both nil))
          (when (and (not (send n1 :get-value :overload?)) (not (send n2 :get-value :overload?)))
              (cond 
               ((or (eq (send n1 :get-value :contact-type) 'grasp) (eq (send n2 :get-value :contact-type) 'grasp)) ;;when either node is grasp knob
                (if (and (equal (send n1 :get-value :object-coords-index) (send n2 :get-value :object-coords-index)) (equal (send n1 :get-value :arm) (send n2 :get-value :arm)))
                    (send graph :add-arc n1 n2
                          (calc-distance-between-pose 
                           (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose))
                           (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose))
                           (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :base-coords-optimized) (send n1 :get-value :base-coords))
                           (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :base-coords-optimized) (send n2 :get-value :base-coords)))
                          :both nil)))
               ((and (equal (send n1 :get-value :arm) (send n2 :get-value :arm)) (equal (send n1 :get-value :contact-point) (send n2 :get-value :contact-point)))
                (if (transitable? n1 n2)
                    (send graph :add-arc n1 n2
                          (calc-distance-between-pose 
                           (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :robot-pose))
                           (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :robot-pose))
                           (if (send n1 :get-value :robot-pose-optimized) (send n1 :get-value :base-coords-optimized) (send n1 :get-value :base-coords))
                           (if (send n2 :get-value :robot-pose-optimized) (send n2 :get-value :base-coords-optimized) (send n2 :get-value :base-coords)))
                          :both nil))
                )
              ))))))))

;;  (clear-zero-relation-nodes graph)
  (dolist (n *bridge-nodes*)
    (let ((ri (get-node (car (send n :get-value :islands))))
          (li (get-node (cadr (send n :get-value :islands)))))
      (when (and (member ri (send graph :nodes)) (member li (send graph :nodes)))
        (send graph :add-node n)
        (send graph :add-arc ri n
              (calc-distance-between-pose
               (if (send ri :get-value :robot-pose-optimized) (send ri :get-value :robot-pose-optimized) (send ri :get-value :robot-pose))
               (send n :get-value :robot-pose)
               (if (send ri :get-value :robot-pose-optimized) (send ri :get-value :base-coords-optimized) (send ri :get-value :base-coords))
               (send n :get-value :base-coords))
              :both t)
        (send graph :add-arc li n 
              (calc-distance-between-pose
               (if (send li :get-value :robot-pose-optimized) (send li :get-value :robot-pose-optimized) (send li :get-value :robot-pose))
               (send n :get-value :robot-pose)
               (if (send li :get-value :robot-pose-optimized) (send li :get-value :base-coords-optimized) (send li :get-value :base-coords))
               (send n :get-value :base-coords))
              :both t))))

  (when (member (get-node "r-0-0-fk") (send graph :nodes))
    (send graph :add-node (car *start-nodes*))
    (send graph :add-arc (car *start-nodes*) (get-node "r-0-0-fk") 1 :both nil))

  (when (member (get-node "l-0-0-fk") (send graph :nodes))
    (send graph :add-node (cadr *start-nodes*))
    (send graph :add-arc (cadr *start-nodes*) (get-node "l-0-0-fk") 1 :both nil))

  (setq *graph* graph)
  (set-start-goal-state)
  ))

(defun solve-graph ()
  (defmethod a*-graph-search-solver
    (:hn (n p)
       0.0))
  (setq *solver* (instance a*-graph-search-solver :init *graph*))
  ;; (setq *solver* (instance depth-first-graph-search-solver))
  (setq *path* (send *solver* :solve *graph* :verbose nil))
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
  ;; (send *graph* :start-state (send *graph* :node "r-0-s-fk"))
  ;; (send *graph* :start-state (send *graph* :node "l-0-s-fk"))
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

(defun play-path (&optional (p *path*) (t-inv 300))
  (let ((ns (send-all p :state))
        (c 0))
    (dolist (n ns)
      (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
        (print n)
        (if (send n :get-value :robot-pose-optimized)
            (play-node n :optimized t)
          (play-node n))
;;        (send *irtviewer* :viewer :viewsurface :write-to-image-file (format nil "~A_~A" c (send n :name)))
        (incf c)
        (unix:usleep (* 1000 t-inv))))))

(defun execute-robot-motion (&optional (p *path*))
  (let ((t-interval 5000.0) ;msec
        (ns (send-all p :state))
        avs
        base-cds-list
        traj-points
        diff-v
        diff-cds
        sttm)

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
    (dotimes (i (length base-cds-list))
      ;; (if (= i 0)
          (setq diff-cds (send (send *pr2* :copy-worldcoords) :transformation (elt base-cds-list i)))
        ;; (setq diff-cds (send (elt base-cds-list (- i 1)) :transformation (elt base-cds-list i))))
          (setq diff-v (send diff-cds :pos))
          (push (float-vector (elt diff-v 0)
                              (elt diff-v 1)
                              (vector-angle #f(1 0 0) (matrix-column (send diff-cds :worldrot) 0) #f(0 0 1)))
                              traj-points))

    (send *ri* :angle-vector (car avs) 1000)
    (unix::usleep (* 1000 1000))
    (send *ri* :wait-interpolation)

    (do-until-key 
     (print "press key")
     (unix::usleep (* 1000 1000)))
    (setq sttm (ros::time+ (ros::time-now) (ros::time 2)))
    (setq *traj-points* traj-points)
    (setq *base-cds-list* base-cds-list)
    (setq *avs* avs)

    (print "check1")
    (send *ri* :angle-vector-sequence avs 
          (make-list (length avs) :initial-element t-interval)
          :default-controller sttm)
    (print "check")
    (send *ri* :move-trajectory-sequence 
          (reverse traj-points)
          (make-list (length base-cds-list) :initial-element t-interval)
          :start-time sttm
          :send-action t)
    ))

