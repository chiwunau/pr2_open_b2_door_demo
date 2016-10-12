(load "extended-graph.l")

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
          (cds-info-seq (send samples :get :cds-info-seq)))

      (dotimes (i (length pose-seq)) ;;number of contact-points
        (dotimes (j (length (elt pose-seq i))) ;;door-angle
          (dotimes (k (length (elt (elt pose-seq i) j))) ;;pose
            (let ((node (instance node :init (format nil "~A-~A-~A-~A" 
                                                     (if (equal samples *filtered-left-samples*) 'l 'r) i j k))))
              (send node :put :arm (if (equal samples *filtered-left-samples*) :larm :rarm))
              (send node :put :contact-point i)
              (send node :put :level j)
              (send node :put :door-angle (* (+ j 1) -20))
              (send node :put :pose-idx k)
              
              (send node :put :pose (elt (elt (elt pose-seq i) j) k))
              (send node :put :base (elt (elt (elt base-seq i) j) k))
              (send node :put :torque (elt (elt (elt torque-seq i) j) k))
              (send node :put :cds-info (elt (elt (elt cds-info-seq i) j) k))
              
              (send node :put :overload? (overload-node? node door-weight))
              
              (push node (elt *sorted-nodes-by-graph-depth* j))
              (push node *all-node*)))))))
  (setq *all-node* (reverse *all-node*)))

(defun redefine-spare-arm ()
  (dolist (n *all-node*)
    (send *pr2* :angle-vector (send n :get :pose))
    (if (equal (send n :get :arm) :larm)
        (send *pr2* :rarm :angle-vector (get-tuckarm :larm :outside :rarm))
      (send *pr2* :larm :angle-vector (get-tuckarm :rarm :outside :larm)))
    (send *irtviewer* :draw-objects :flush t)
    (send n :put :pose (send *pr2* :angle-vector))))



(defun filter-out-nodes (&optional (door-weight 2.5))
  (setq *filtered-nodes* nil)
  (dolist (ns *sorted-nodes-by-graph-depth*)
    (dolist (n ns)
      (check-node-overload n door-weight)
      )))

(defun overload-node? (n &optional (door-weight 2.5))
  (let* ((force-cds (elt (send n :get :cds-info) 0))
         ;;             (contact-pt (elt (send n :get :cds-info) 2))
         (contact-pt (elt (reverse (set-contact-points (send n :get :arm))) (send n :get :contact-point)))
         (max-tq (send *pr2* :max-torque-vector))
         (overload nil)
         drw tq)
    (send *pr2* :angle-vector (send n :get :pose))
    (send *pr2* :move-to (send n :get :base) :world)
    (send *door* :hinge :joint-angle (send n :get :door-angle))
    
    (setq drw (door-reaction-wrench force-cds contact-pt door-weight))
    (setq tq (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords (list contact-pt)))
    
    (dotimes (i (length max-tq))
      (if (> 0 (- (abs (elt max-tq i)) (abs (elt tq i))))
          (setq overload t)))

    overload))



(defun init-graph ()
  (setq *graph* (instance costed-graph))
  (dolist (n *all-node*)
    (unless (send n :get :overload?)
      (send *graph* :add-node n)))
  (dotimes (lv (- *graph-depth* 1))
    (let ((ns1 (elt *sorted-nodes-by-graph-depth* lv))
          (ns2 (elt *sorted-nodes-by-graph-depth* (+ lv 1))))
      (dolist (n1 ns1)
        (dolist (n2 ns2)
          (when (and (not (send n1 :get :overload?)) (not (send n2 :get :overload?)) (equal (send n1 :get :arm) (send n2 :get :arm)))
            (send *graph* :add-arc n1 n2 
                  (calc-smoothness-between-pose  ;;cost of arc
                   (send n1 :get :pose) (send n2 :get :pose) (send n1 :get :base) (send n2 :get :base))
                  :both nil)))))))
          
        
(defun solve-graph ()
  (defmethod a*-graph-search-solver
    (:hn (n p)
       0.0))
  (setq *solver* (instance a*-graph-search-solver :init *graph*))
  (setq *path* (send *solver* :solve *graph* :verbose nil))
  (setq *m* (list (send-all (send-all *path* :state) :get :pose) (send-all (send-all *path* :state) :get :base) (send-all (send-all *path* :state) :get :torque)))
  )
  

(defun set-start-goal-state ()  
  (send *graph* :start-state (send *graph* :node "l-0-0-2"))
  (send *graph* :goal-state (send *graph* :node "l-0-4-7"))
)

