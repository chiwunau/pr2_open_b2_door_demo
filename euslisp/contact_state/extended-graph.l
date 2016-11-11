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
                      (dolist (target-node nodes)
                        (format f "   ")
                        ;; hoge = "hoge";                 
                        (format f "~A [label = \"~A\"];~%"
                                (graphviz-node-name target-node)
                                (send target-node :name))
                        )
                      ;; first of all, write result-path
                      (let ((drawed-arc nil))
                        (let (anode)
                          (while (and (setq anode (pop result-path)) result-path)
                            (format f "   ")
                            (format f "~A -> ~A [color = red];~%"
                                    (graphviz-node-name (send anode :state))
                                    (graphviz-node-name (send (car result-path) :state)))))
                        (dolist (target-node nodes)
                          (let ((neighbors
                                 (mapcar #'cdr (send self :successors target-node))))
                            (dolist (neighbor neighbors)
                              (when (not (or (find (cons target-node neighbor) drawed-arc
                                                   :test #'equal)
                                             (find (cons neighbor target-node) drawed-arc
                                                   :test #'equal)))
                                (if (member target-node
                                            (mapcar #'cdr (send self :successors neighbor)))
                                    (progn
                                      (format f "   ")
                                      (format f "~A -> ~A [dir = both];~%"
                                              (graphviz-node-name target-node)
                                              (graphviz-node-name neighbor)))
                                  (progn
                                    (format f "   ")
                                    (format f "~A -> ~A;~%"
                                            (graphviz-node-name target-node)
                                            (graphviz-node-name neighbor))))
                                (push (cons target-node neighbor) drawed-arc)))))
                        (format f "}~%")
                        t)))))
  )

(defclass extended-node
  :super node
  :slots (arm islands contact-point-index contact-point contact-type graph-depth door-angle solved robot-pose robot-pose-optimized base-coords base-coords-optimized object-coords-index object-coords ik-target-coords torque overload? transitable-nodes intransitable-nodes))
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
         (:overload?
          overload?)
         )
   )
)
;; (defmethod graph-search-solver
;;   (:solve (prblm &key (verbose nil))
;;     (send self :solve-init prblm)
;;     (while (not (send self :null-open-list?))
;; ;;;       (if verbose
;; ;;;           (warn "current open-list num -> ~A -- ~A --~%"
;; ;;;                 (length open-list)
;; ;;;                 :solve))
;;       (let ((target-node (send self :pop-from-open-list :debug verbose)))
;;         ;; here, target-node is removed from open-list in :pop-from-open-list.
;; ;;;         (if verbose
;; ;;;             (warn "target-node is ~A -- ~A -- ~%"
;; ;;;                   (send target-node :state) :solve))
;;         (cond ((send prblm :goal-test (send target-node :state))
;; ;;;                (if verbose
;; ;;;                    (warn "arrived at goal! -- ~A --~%" :solve))
;;                (return-from :solve (send target-node :path)))
;;               ((not (send self :find-node-in-close-list target-node))
;;                (push (send target-node :state) close-list)
;;                (send self :add-list-to-open-list
;;                      (send target-node :expand prblm :verbose verbose)))
;;               )))
;;     (warn "open-list is nil... -- ~A --~%" :solve)
;;     (warn "search was missed!! -- ~A --~%" :solve)
;;     (send self :clear-open-list)
;;     (setq close-list nil)
;;     nil)
;; )


;; (defmethod best-first-graph-search-solver
;;   (:pop-from-open-list (&key (debug nil))
;;     ;; returns the element which has minimum priority-value.
;;     ;;      => Priority Queue.
;;     ;; priority-value is calculated by :fn.
;;     ;; In AIMA , priority-value is equal to f(n).
;;     ;;
;;     ;; elements of open-list must be  propertied-objects
;; ;;                       (push (copy-object open-list) *open-list*)
;;     (let* ((min-x (car open-list)))
;;       (let ((min-value (or (send min-x :get :priority-value)
;;                            (progn   ; if does not have :priority-value
;;                              (send min-x :put :priority-value
;;                                    (send self :fn min-x aproblem))
;;                              (send min-x :get :priority-value)))))
;;         (when debug
;;           (warn "<~A> v -> ~A~%"
;;                 (send-all (send-all (send min-x :path)  :state) :name)
;;                 min-value))
;;         (dolist (x (cdr open-list))
;;           (let ((v (or (get x :priority-value)
;;                        (progn       ; if does not have :priority-value
;;                          (setf (get x :priority-value)
;;                                (send self :fn x aproblem))
;;                          (get x :priority-value)))))
;;             (when debug
;;               (warn "<~A> v -> ~A~%"
;;                     (send-all (send-all (send x :path) :state) :name)
;;                     v))
;;             (if (< v min-value)
;;                 (progn
;;                   (setq min-value v)
;;                   (setq min-x x)))
;;             ))
;;   ;;      (push min-x *min-x*)
;;       (when debug
;;         (warn ":pop-from-open-list result...~%")
;;         (warn "min-value -> ~A~%" min-value)
;;         (warn "min-x -> ~A~%"
;;               (send-all (send-all (send min-x :path) :state) :name)))
;;       ;; remove from open-list
;;       (setq open-list (delete min-x open-list :count 1))
;;       min-x)))

;; ;;   (:pop-from-open-list 
;; ;;    (prblm &key (debug nil))
;; ;;     ;; returns the element which has minimum priority-value.
;; ;;     ;;      => Priority Queue.
;; ;;     ;; priority-value is calculated by :fn.
;; ;;     ;; In AIMA , priority-value is equal to f(n).
;; ;;     ;;
;; ;;     ;; elements of open-list must be  propertied-objects
;; ;; ;;---------------------------------------------------------------------------------------------------------------
;; ;;    (push (copy-object open-list) *open-list*)
;; ;;    (let (intrasitable-x-list) ;;make sure first element in open-list work
;; ;;        (dolist (x open-list)
;; ;; ;;         (print (send x :state))
;; ;;          (if (equal (send x :state) (send prblm :start-state))
;; ;;              (return))
;; ;;          (let ((x-transitable (find (send x :state) (send (send (send x :parent) :state) :transitable-nodes)))
;; ;;                (x-intransitable (find (send x :state) (send (send (send x :parent) :state) :intransitable-nodes))))
           
;; ;;            (if x-transitable
;; ;;                (return)
;; ;;              (if x-intransitable
;; ;;                  (push x intrasitable-x-list)
;; ;;                (if (transitable? (send (send x :parent) :state) (send x :state))
;; ;;                    (send (send (send x :parent) :state) :put-transitable-node (send x :state))
;; ;;                  (progn (send (send (send x :parent) :state) :put-intransitable-node (send x :state))
;; ;;                         (push x intrasitable-x-list)))))))
;; ;;        (dolist (in-x intrasitable-x-list)
;; ;;          (setq open-list (delete in-x open-list :count 1))))

;; ;;    (unless open-list
;; ;;      (return-from :pop-from-open-list nil))
       
;; ;;     (let* ((min-x (car open-list)))
;; ;;       (let ((min-value (or (send min-x :get :priority-value)
;; ;;                            (progn   ; if does not have :priority-value
;; ;;                              (send min-x :put :priority-value
;; ;;                                    (send self :fn min-x aproblem))
;; ;;                              (send min-x :get :priority-value)))))
;; ;;         (when debug
;; ;;           (warn "<~A> v -> ~A~%"
;; ;;                 (send-all (send-all (send min-x :path)  :state) :name)
;; ;;                 min-value))

;; ;;         (let (intrasitable-x-list)
;; ;;           (dolist (x (cdr open-list))
;; ;;             (let ((v (or (get x :priority-value)
;; ;;                          (progn       ; if does not have :priority-value
;; ;;                            (setf (get x :priority-value)
;; ;;                                  (send self :fn x aproblem))
;; ;;                            (get x :priority-value))))
;; ;;                   (x-transitable (find (send x :state) (send (send (send x :parent) :state) :transitable-nodes)))
;; ;;                   (x-intransitable (find (send x :state) (send (send (send x :parent) :state) :intransitable-nodes))))

;; ;;             (when debug
;; ;;               (warn "<~A> v -> ~A~%"
;; ;;                     (send-all (send-all (send x :path) :state) :name)
;; ;;                     v))
            
;; ;;               (if x-transitable
;; ;;                   (if (and (< v min-value))
;; ;;                       (progn
;; ;;                         (setq min-value v)
;; ;;                         (setq min-x x)))
;; ;;                 (if x-intransitable
;; ;;                     (push x intrasitable-x-list)
;; ;;                   (if (transitable? (send (send x :parent) :state) (send x :state))
;; ;;                       (progn (send (send (send x :parent) :state) :put-transitable-node (send x :state))
;; ;;                              (if (and (< v min-value))
;; ;;                                  (progn
;; ;;                                    (setq min-value v)
;; ;;                                    (setq min-x x))))
;; ;;                     (progn (send (send (send x :parent) :state) :put-intransitable-node (send x :state))
;; ;;                            (push x intrasitable-x-list)))))))
;; ;;           (dolist (in-x intrasitable-x-list)
;; ;;             (setq open-list (delete in-x open-list :count 1))))

;; ;;         (push min-x *min-x*)
;; ;;       (when debug
;; ;;         (warn ":pop-from-open-list result...~%")
;; ;;         (warn "min-value -> ~A~%" min-value)
;; ;;         (warn "min-x -> ~A~%"
;; ;;               (send-all (send-all (send min-x :path) :state) :name)))
;; ;;       ;; remove from open-list
;; ;;       (setq open-list (delete min-x open-list :count 1))
;; ;;       min-x)))
;; )
