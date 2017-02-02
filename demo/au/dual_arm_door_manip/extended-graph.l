(defmethod costed-graph
  (:start-state (&optional arg)
                (if arg (setq start-state arg))
                start-state)
  (:goal-state (&optional arg)
               (if arg (setq goal-state arg))
               goal-state)
  (:goal-test (gs)
              (eq goal-state gs))
  
  (:write-to-dot (fname &optional result-path (title "output"))
                 (let ((node-alist          ; ((node . symbol) (node . symbol) ...)
                        (mapcar #'(lambda (n)
                                    (cons n (string (gensym))))
                                nodes)))
                   (labels ((graphviz-node-name
                             (n)
                             (cdr (assoc n node-alist))))
                     (with-open-file 
                      (f fname :direction :output)
                      (format f "digraph ~A {~%" title)
;;                     (format f "size = \"8.3, 11.7\";~%")
                      (format f "ratio = ~A;~%" (/ 8.3 11.7))
                       ;; (format f "ratio = 1;~%")
                     ;; (format f "ranksep = \"3.0 equally\";~%")
                     (format f "nodesep = 0.05;~%")

                      (dolist (target-node nodes)
                        (format f "   ")
                        ;; hoge = "hoge";                 
                        ;; (format f "~A [label = \"~A\"];~%"
                        ;;         (graphviz-node-name target-node)
                        ;;         (send target-node :name))
                        (unless (send target-node :get-value :islands)
                          (format f "~A [label = \"~A\", fontsize=100];~%"
                                  (graphviz-node-name target-node)
                                  ;; (if (send target-node :get-value :islands)
                                  ;;     (let ((a (car (send target-node :get-value :islands)))
                                  ;;           (b (cadr (send target-node :get-value :islands))))
                                  ;;       ;; (format nil "~A\\n~A" a b))
                                  ;;       (format nil "bridge"))
                                  (send target-node :name))
                        (if (member target-node (send-all result-path :state))
                            (format f "~A [color =red];~%"
                                    (graphviz-node-name target-node)))
                        )
                        )
                      ;; first of all, write result-path
                      (let ((drawed-arc nil))
                        ;; (let (anode)
                        ;;   (while (and (setq anode (pop result-path)) result-path)
                        ;;     (format f "   ")
                        ;;     (format f "~A -> ~A [color = red];~%"
                        ;;             (graphviz-node-name (send anode :state))
                        ;;             (graphviz-node-name (send (car result-path) :state)))))
                        (dolist (target-node nodes)
                          (let ((neighbors
                                 (mapcar #'cdr (send self :successors target-node))))
                            (dolist (neighbor neighbors)
                              (when (not (or (find (cons target-node neighbor) drawed-arc
                                                   :test #'equal)
                                             (find (cons neighbor target-node) drawed-arc
                                                   :test #'equal)
                                             (and (send target-node :get-value :islands)
                                                  (or (find (cons (send self :node (car (send target-node :get-value :islands))) (send self :node (cadr (send target-node :get-value :islands)))) drawed-arc
                                                        :test #'equal)
                                                      (find (cons (send self :node (cadr (send target-node :get-value :islands))) (send self :node (car (send target-node :get-value :islands)))) drawed-arc
                                                            :test #'equal)))
                                             (send neighbor :get-value :islands)
                                             ))
                                (if (member target-node
                                            (mapcar #'cdr (send self :successors neighbor))) ;;both arc
                                    (progn
                                      (format f "   ")
                                      (cond 
                                       ((send target-node :get-value :islands)
                                        (if (member target-node (send-all result-path :state))
                                            (if (< (position (send self :node (car (send target-node :get-value :islands))) (send-all result-path :state)) (position (send self :node (cadr (send target-node :get-value :islands))) (send-all result-path :state)))
                                                (format f "~A -> ~A [color = red, style = dashed];~%"
                                                        (graphviz-node-name (send self :node (car (send target-node :get-value :islands))))
                                                        (graphviz-node-name (send self :node (cadr (send target-node :get-value :islands)))))
                                              (format f "~A -> ~A [color = red, style = dashed];~%"
                                                      (graphviz-node-name (send self :node (cadr (send target-node :get-value :islands))))
                                                      (graphviz-node-name (send self :node (car (send target-node :get-value :islands))))))
                                          (format f "~A -> ~A [dir = both, color = green, style = bold];~%"
                                                  (graphviz-node-name (send self :node (car (send target-node :get-value :islands))))
                                                  (graphviz-node-name (send self :node (cadr (send target-node :get-value :islands))))))
                                        (push (cons (send self :node (car (send target-node :get-value :islands))) (send self :node (cadr (send target-node :get-value :islands)))) drawed-arc))
                                       ((and (member target-node (send-all result-path :state)) (member neighbor (send-all result-path :state)) (eq 1 (abs (- (position target-node (send-all result-path :state)) (position neighbor (send-all result-path :state))))))
                                            (if (< (position target-node (send-all result-path :state)) (position neighbor (send-all result-path :state)))
                                                (format f "~A -> ~A [style = bold, color = red];~%"
                                                        (graphviz-node-name target-node)
                                                        (graphviz-node-name neighbor))
                                              (format f "~A -> ~A [style = bold, color = red];~%"
                                                      (graphviz-node-name neighbor)
                                                      (graphviz-node-name target-node)))
                                            (push (cons target-node neighbor) drawed-arc))
                                       (t
                                        (format f "~A -> ~A [dir = both];~%"
                                                  (graphviz-node-name target-node)
                                                  (graphviz-node-name neighbor))
                                        (push (cons target-node neighbor) drawed-arc))))
                                  (progn
                                    (format f "   ")
                                    (if (and (member target-node (send-all result-path :state)) (member neighbor (send-all result-path :state)) (eq 1 (abs (- (position target-node (send-all result-path :state)) (position neighbor (send-all result-path :state))))))
                                        (format f "~A -> ~A [style = bold, color = red];~%"
                                                (graphviz-node-name target-node)
                                                (graphviz-node-name neighbor))
                                      (format f "~A -> ~A;~%"
                                              (graphviz-node-name target-node)
                                              (graphviz-node-name neighbor)))
                                    (push (cons target-node neighbor) drawed-arc)))
                                
                                ))))
                        (format f "}~%")
                        t)))))
  ;; (:write-to-pdf
  ;;  (fname &optional (title "default_title") result-path)
  ;;  (let ((dot-fname
  ;;         (format nil "~A.dot" fname)))
  ;;    (send self :write-to-dot dot-fname result-path title)
  ;;    (unix::system (format nil "dot -Tpdf ~a -o ~a-dot.pdf" dot-fname fname))
  ;;    (unix::system (format nil "neato -Tpdf ~a -o ~a-neato.pdf" dot-fname fname))
  ;;    (unix::system (format nil "fdp -n -Gsplines=true -Tpdf ~a -o ~a-fdp.pdf" dot-fname fname))
  ;;    (unix::system (format nil "twopi -n -Gsplines=true -Tpdf ~a -o ~a-twopi.pdf" dot-fname fname))
  ;;    t))
  (:write-to-png
   (fname &optional (title "default_title") result-path)
   (let ((dot-fname
          (format nil "~A.dot" fname)))
     (send self :write-to-dot dot-fname result-path title)
     (unix::system (format nil "dot -Tpng ~a -o ~a-dot.png" dot-fname fname))
     ;; (unix::system (format nil "neato -Tpng ~a -o ~a-neato.png" dot-fname fname))
     ;; (unix::system (format nil "fdp -n -Gsplines=true -Tpng ~a -o ~a-fdp.png" dot-fname fname))
     ;; (unix::system (format nil "twopi -n -Gsplines=true -Tpng ~a -o ~a-twopi.png" dot-fname fname))
     t))
  )

(defclass extended-node
  :super node
  :slots (arm islands contact-point-index contact-point contact-type graph-depth door-angle solved robot-pose robot-pose-optimized base-coords base-coords-optimized object-coords-index object-coords ik-target-coords torque torque-optimized overload? transitable-nodes intransitable-nodes))
(defmethod extended-node
  (:transitable-nodes 
   ()
   transitable-nodes
   )
  (:intransitable-nodes 
   ()
   intransitable-nodes
   )
  (:put-transitable-node 
   (n)
   (if (find n transitable-nodes)
       (warn "~A already in the list~%" n)
     (push n transitable-nodes)))
  (:put-intransitable-node 
   (n)
   (if (find n intransitable-nodes)
       (warn "~A already in the list~%" n)
     (push n intransitable-nodes)))

  (:put-value 
   (key val)
   (case key
         (:arm
          (setq arm val))
         (:islands ;;for mutiple point contact
          (setq islands val))
         (:contact-point-index
          (setq contact-point-index val))
         (:contact-point
          (setq contact-point val))
         (:contact-type
          (setq contact-type val))
         (:graph-depth
          (setq graph-depth val))
         (:door-angle
          (setq door-angle val))
         (:solved
          (setq solved val))
         (:robot-pose
          (setq robot-pose val))
         (:robot-pose-optimized
          (setq robot-pose-optimized val))
         (:base-coords
          (setq base-coords val))
         (:base-coords-optimized
          (setq base-coords-optimized val))
         (:object-coords-index
          (setq object-coords-index val))
         (:object-coords
          (setq object-coords val))
         (:ik-target-coords
          (setq ik-target-coords val))
         (:torque
          (setq torque val))
         (:torque-optimized
          (setq torque-optimized val))
         (:overload?
          (setq overload? val))
         )
   )

  (:get-value
   (key)
   (case key
         (:arm
          arm)
         (:islands ;;for mutiple point contact
          islands)
         (:contact-point-index
          contact-point-index)
         (:contact-point
          contact-point)
         (:contact-type
          contact-type)
         (:graph-depth
          graph-depth)
         (:door-angle
          door-angle)
         (:solved
          solved)
         (:robot-pose
            robot-pose)
         (:robot-pose-optimized
          robot-pose-optimized)
         (:base-coords
          base-coords)
         (:base-coords-optimized
          base-coords-optimized)
         (:object-coords-index
          object-coords-index)
         (:object-coords
          object-coords)
         (:ik-target-coords
          ik-target-coords)
         (:torque
          torque)
         (:torque-optimized
          torque-optimized)
         (:overload?
          overload?)
         )
   )
)
