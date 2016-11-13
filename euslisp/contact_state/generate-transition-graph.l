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
          ;; (when (and (not (send n1 :get-value :overload?)) (not (send n2 :get-value :overload?)) (equal (send n1 :get-value :arm) (send n2 :get-value :arm)))
              (cond 
               ((and (equal (send n1 :get-value :arm) (send n2 :get-value :arm)) (equal (send n1 :get-value :contact-point) (send n2 :get-value :contact-point)))

                (if (transitable? n1 n2)
                    (send graph :add-arc n1 n2
                          (calc-distance-between-pose (send n1 :get-value :robot-pose) (send n2 :get-value :robot-pose) (send n1 :get-value :base-coords) (send n2 :get-value :base-coords)) ;;cost of arc
                    :both nil))
                ))))))))

  (dolist (n *bridge-nodes*)
    (let ((ri (get-node (car (send n :get-value :islands))))
          (li (get-node (cadr (send n :get-value :islands)))))
      (when (and (member ri (send graph :nodes)) (member li (send graph :nodes)))
        (send graph :add-node n)
        (send graph :add-arc ri n 0 :both t)
        (send graph :add-arc li n 0 :both t))))

   (setq *graph* graph)
   (set-start-goal-state)
  ))

(defun solve-graph ()
  (defmethod a*-graph-search-solver
    (:hn (n p)
       0.0))
  (setq *solver* (instance a*-graph-search-solver :init *graph*))
  (setq *path* (send *solver* :solve *graph* :verbose t))
  (let ((pa (reverse (cdr (reverse (cdr *path*))))))
    (setq *m* (list (send-all (send-all pa :state) :get-value :robot-pose) (send-all (send-all pa :state) :get-value :base-coords) (send *pr2* :max-torque-vector)))
    ))

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
