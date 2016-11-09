(load "extended-graph.l")
(load "graph-rule.l")

(setq *all-node* nil)
(setq *filtered-nodes* nil)
(setq *graph* nil)
(setq *sorted-nodes-by-graph-depth* (make-list *graph-depth*))

(defun init-all-node (&key (door-weight 2.5))
  (setq *all-node* nil)
  (setq *sorted-nodes-by-graph-depth* (make-list *graph-depth*))
  (dolist (samples (list *filtered-left-samples* *filtered-right-samples*))
    (let ((pose-seq (send samples :get :pose-seq))
          (base-seq (send samples :get :base-seq))
          (torque-seq (send samples :get :torque-seq))
          (cds-info-seq (send samples :get :cds-info-seq))
          (contact-pts (set-contact-points (if (equal samples *filtered-left-samples*) :larm :rarm))))

      (dotimes (i (length pose-seq)) ;;number of contact-points
        (dotimes (j (length (elt pose-seq i))) ;;door-angle
          (dotimes (k (length (elt (elt pose-seq i) j))) ;;pose
            (let ((node (instance node :init (format nil "~A-~A-~A-~A"
                                                     (if (equal samples *filtered-left-samples*) 'l 'r) i j k))))
              (send node :put :arm (if (equal samples *filtered-left-samples*) :larm :rarm))
              (send node :put :contact-point-idx i)
              (send node :put :contact-point (elt contact-pts (- (length pose-seq) i 1)))
              (send node :put :level j)
              (send node :put :door-angle (* (+ j 1) -20))
              (send node :put :pose-idx k)
              (send node :put :pose (elt (elt (elt pose-seq i) j) k))
              (send node :put :base (elt (elt (elt base-seq i) j) k))
              (send node :put :cds-info (elt (elt (elt cds-info-seq i) j) k)) ;;cds-on-plane, shift-cds, contact-pt-cds
              (send node :put :torque (calc-node-torque node door-weight)) ;;calculate door torque in this weight, can be simplified (TODO) by simplily multifying
              (send node :put :overload? (overload-node? node :ratio 1))

;;              (send node :put :torque (elt (elt (elt torque-seq i) j) k))

              (push node (elt *sorted-nodes-by-graph-depth* j))
              (push node *all-node*)))))))
  (setq *all-node* (reverse *all-node*))
  (redefine-spare-arm))

(defun redefine-spare-arm ()
  (dolist (n *all-node*)
    (send *pr2* :angle-vector (send n :get :pose))
    (if (equal (send n :get :arm) :larm)
        (send *pr2* :rarm :angle-vector (get-tuckarm :larm :outside :rarm))
      (send *pr2* :larm :angle-vector (get-tuckarm :rarm :outside :larm)))
    ;; (send *irtviewer* :draw-objects :flush t)
    (send n :put :pose (send *pr2* :angle-vector))))



(defun filter-out-nodes (&optional (door-weight 2.5))
  (setq *filtered-nodes* nil)
  (dolist (ns *sorted-nodes-by-graph-depth*)
    (dolist (n ns)
      (check-node-overload n door-weight)
      )))

(defun calc-node-torque (n &optional (door-weight 2.5))
  (let* ((contact-pt (send n :get :contact-point))
         drw tq)
    (send *pr2* :angle-vector (send n :get :pose))
    (send *pr2* :move-to (send n :get :base) :world)
    (send *door* :hinge :joint-angle (send n :get :door-angle))
    
    (setq drw (door-reaction-wrench (elt (send n :get :cds-info) 0) (make-coords) :door-weight door-weight))
    (let ((wrenches-tgt (make-cascoords :pos (copy-object (send contact-pt :worldpos)) :rot (copy-object (send (elt (send n :get :cds-info) 1) :worldrot)))))
      (send (send contact-pt :parent) :assoc wrenches-tgt)
      (setq tq (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords (list wrenches-tgt))))
    tq))

(defun overload-node? (n &key (ratio 1))
  (let* ((max-tq (send *pr2* :max-torque-vector))
         (overload nil)
         (tq (send n :get :torque)))
    
    (dotimes (i (length max-tq))
      (if (> 0 (- (* ratio (abs (elt max-tq i))) (abs (elt tq i))))
          (setq overload t)))
    overload))



(defun init-graph ()
  (setq *graph* (instance costed-graph :init))
  (setq *start-node* (instance node :init "start"))
  (setq *end-node* (instance node :init "end"))

  (send-all *all-node* :remove-all-arcs)
  (send *start-node* :remove-all-arcs)
  (send *end-node* :remove-all-arcs)
  
  (send *graph* :add-node *start-node*)
  (send *graph* :add-node *end-node*)
  ;;add nodes to graph
  (dolist (n *all-node*)
    (unless (send n :get :overload?)
      (send *graph* :add-node n)))

  ;;add arcs to graph
  (dotimes (lv (- *graph-depth* 1))
    (let ((ns1 (elt *sorted-nodes-by-graph-depth* lv))
          (ns2 (elt *sorted-nodes-by-graph-depth* (+ lv 1))))
      (dolist (n1 ns1)
        (when (member n1 (send *graph* :nodes))
          (if (equal (send n1 :get :level) 0) ;;add start node
              (send *graph* :add-arc *start-node* n1 1 :both nil))
          (dolist (n2 ns2)
            (when (member n2 (send *graph* :nodes))
              (if (and (not (member *end-node* (send n2 :neighbors))) (equal (send n2 :get :level) (- *graph-depth* 1))) ;;add end node
                    (send *graph* :add-arc n2 *end-node* 1 :both nil))
          ;; (when (and (not (send n1 :get :overload?)) (not (send n2 :get :overload?)) (equal (send n1 :get :arm) (send n2 :get :arm)))
              (cond 
               ((and (equal (send n1 :get :arm) (send n2 :get :arm)) (equal (send n1 :get :contact-point) (send n2 :get :contact-point)))
               ;; ((equal (send n1 :get :arm) (send n2 :get :arm))

;                (print (transitable? n1 n2))
                (if (transitable? n1 n2)
                    (send *graph* :add-arc n1 n2
                          (calc-distance-between-pose (send n1 :get :pose) (send n2 :get :pose) (send n1 :get :base) (send n2 :get :base)) ;;cost of arc
                    :both nil))))))))))
  (set-start-goal-state))

(defun solve-graph ()
  (defmethod a*-graph-search-solver
    (:hn (n p)
       0.0))
  (setq *solver* (instance a*-graph-search-solver :init *graph*))
  (setq *path* (send *solver* :solve *graph* :verbose nil))
  (let ((pa (reverse (cdr (reverse (cdr *path*))))))
    (setq *m* (list (send-all (send-all pa :state) :get :pose) (send-all (send-all pa :state) :get :base) (send-all (send-all pa :state) :get :torque)))
    ))

(defun set-start-goal-state ()
  (send *graph* :start-state (send *graph* :node "start"))
  (send *graph* :goal-state (send *graph* :node "end"))
)

(defun play-node (n)
  (send *pr2* :angle-vector (send n :get :pose))
  (send *pr2* :move-to (send n :get :base) :world)
  (send *door* :hinge :joint-angle (send n :get :door-angle))
  (send *irtviewer* :draw-objects :flush t)
  (send *pr2* :draw-torque (send *irtviewer* :viewer) :torque-vector (scale 5 (send n :get :torque)))
  (send *irtviewer* :viewer :viewsurface :flush))
              
