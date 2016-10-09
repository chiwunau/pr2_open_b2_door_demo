(setq *all-node* nil)
(setq *graph* nil)
(setq *sorted-nodes-by-graph-depth* (make-list *graph-depth*))

(defun init-all-node ()
  (setq *all-node* nil)
  (setq *sorted-nodes-by-graph-depth* (make-list *graph-depth*))
  (dolist (samples (list *filtered-left-samples*)) ;;*filtered-right-samples*))
    (let ((pose-seq (send samples :get :pose-seq))
          (base-seq (send samples :get :base-seq))
          (torque-seq (send samples :get :torque-seq)))
      (dotimes (i (length pose-seq)) ;;number of contact-points
        (dotimes (j (length (elt pose-seq i))) ;;door-angle
          (dotimes (k (length (elt (elt pose-seq i) j))) ;;pose
            (let ((node (instance node :init (format nil "~A-~A-~A-~A" 
                                                     (if (equal samples *filtered-left-samples*) 'l 'r) i j k))))
              (send node :put :arm (if (equal samples *filtered-left-samples*) :left :right))
              (send node :put :contact-point i)
              (send node :put :level j)
              (send node :put :door-angle (* (+ j 1) -20))
              (send node :put :pose-idx k)
              
              (send node :put :pose (elt (elt (elt pose-seq i) j) k))
              (send node :put :base (elt (elt (elt base-seq i) j) k))
              (send node :put :torque (elt (elt (elt torque-seq i) j) k))
              
              (push node (elt *sorted-nodes-by-graph-depth* j))
              (push node *all-node*)))))))
  (setq *all-node* (reverse *all-node*)))


(defun init-graph ()
  (setq *graph* (instance costed-graph))
  (dolist (n *all-node*)
    (send *graph* :add-node n))
  (dotimes (lv (- *graph-depth* 1))
    (let ((ns1 (elt *sorted-nodes-by-graph-depth* lv))
          (ns2 (elt *sorted-nodes-by-graph-depth* (+ lv 1))))
      (dolist (n1 ns1)
        (dolist (n2 ns2)
          (when (equal (send n1 :get :arm) (send n2 :get :arm))
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
  
  

      
    
    
  
  

  
  

              
            
          
      
     
      
    
  
