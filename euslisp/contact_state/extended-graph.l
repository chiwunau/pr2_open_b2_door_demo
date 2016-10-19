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

  ;; (:write-to-dot
  ;;  (fname &optional result-path (title "output"))
  ;;  (let ((node-alist          ; ((node . symbol) (node . symbol) ...)
  ;;         (mapcar #'(lambda (n)
  ;;                     (cons n (string (gensym))))
  ;;                 nodes)))
  ;;    (labels ((graphviz-node-name
  ;;              (n)
  ;;              (cdr (assoc n node-alist))))
  ;;      (with-open-file
  ;;       (f fname :direction :output)
  ;;       (format f "digraph ~A {~%" title)
  ;;       ;; ;; graph global setting
  ;;       ;; (format f "   overlap = false;~%")
  ;;       ;; (format f "   ranksep = 0.1;~%")
  ;;       ;; (format f "   node [fontname=\"MS Gothic\"];~%")
  ;;       ;; ;; graph label setting
  ;;       ;; (when (send self :name)
  ;;       ;;   (format f "   label=\"~a\";~%" (send self :name))
  ;;       ;;   (format f "   labelloc=top;~%")
  ;;       ;;   (format f "   labeljust=left;~%"))
  ;;       (dolist (target-node nodes)
  ;;         (format f "   ")
  ;;         (format f "~A [label = \"hello~s\"];~%"
  ;;                 (graphviz-node-name target-node)
  ;;                 (send target-node :name))
  ;;         ;; hoge = "hoge";
  ;;         ;; (format f "~a[label=\"~a\", ~a];~%"
  ;;         ;;         (send target-node :name)
  ;;         ;;         (send target-node :get :label)
  ;;         ;;         (cond ((or (equal (send target-node :name) "start") (equal (send target-node :name) "goal"))
  ;;         ;;                "shape=oval, color=red")
  ;;         ;;               ((send target-node :get :image-name)
  ;;         ;;                (format nil "shape=box, image=\"~a\"" (send target-node :get :image-name)))))
  ;;         )
  ;;       ;; first of all, write result-path
  ;;       (let ((drawed-arc nil))
  ;;         (let (anode)
  ;;           (while (and (setq anode (pop result-path)) result-path)
  ;;             (format f "   ")
  ;;             (format f "~A -> ~A [color = red];~%"
  ;;                     (graphviz-node-name (send anode :state))
  ;;                     (graphviz-node-name (send (car result-path) :state)))))
  ;;         (dolist (target-node nodes)
  ;;           (let ((neighbors
  ;;                  (mapcar #'cdr (send self :successors target-node))))
  ;;             (dolist (neighbor neighbors)
  ;;               (when (not (or (find (cons target-node neighbor) drawed-arc
  ;;                                    :test #'equal)
  ;;                              (find (cons neighbor target-node) drawed-arc
  ;;                                    :test #'equal)))
  ;;                 (if (member target-node
  ;;                             (mapcar #'cdr (send self :successors neighbor)))
  ;;                     (progn
  ;;                       (format f "   ")
  ;;                       (format f "~A -> ~A [dir = both];~%"
  ;;                               (send target-node :name)
  ;;                               (send neighbor :name)))
  ;;                   (progn
  ;;                     (format f "   ")
  ;;                     (format f "~A -> ~A;~%"
  ;;                             (send target-node :name)
  ;;                             (send neighbor :name))))
  ;;                 (push (cons target-node neighbor) drawed-arc)))))
  ;;         (format f "}~%")
  ;;         t)))))
  ;; (:write-to-pdf
  ;;  (fname &optional result-path)
  ;;  (let ((dot-fname
  ;;         (format nil "~A.dot" fname)))
  ;;    (send self :write-to-dot dot-fname result-path)
  ;;    (unix::system (format nil "dot -Tpdf ~a -o ~a-dot.pdf" dot-fname fname))
  ;;    (unix::system (format nil "neato -Tpdf ~a -o ~a-neato.pdf" dot-fname fname))
  ;;    (unix::system (format nil "fdp -n -Gsplines=true -Tpdf ~a -o ~a-fdp.pdf" dot-fname fname))
  ;;    (unix::system (format nil "twopi -n -Gsplines=true -Tpdf ~a -o ~a-twopi.pdf" dot-fname fname))
  ;;    t))
  ;; (:write-to-png
  ;;  (fname &optional (title "default_title") result-path)
  ;;  (let ((dot-fname
  ;;         (format nil "~A.dot" fname)))
  ;;    (send self :write-to-dot dot-fname result-path title)
  ;;    (unix::system (format nil "dot -Tpng ~a -o ~a-dot.png" dot-fname fname))
  ;;    (unix::system (format nil "neato -Tpng ~a -o ~a-neato.png" dot-fname fname))
  ;;    (unix::system (format nil "fdp -n -Gsplines=true -Tpng ~a -o ~a-fdp.png" dot-fname fname))
  ;;    (unix::system (format nil "twopi -n -Gsplines=true -Tpng ~a -o ~a-twopi.png" dot-fname fname))
  ;;    t))
  )

