
(defun max-smoothness-motion 
  (&optional (arm :rarm) (idx 0))
  (let* ((pose-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :pose-seq) idx))
         (base-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :base-seq) idx))
         (torque-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :torque-seq) idx))
         st-p st-b
         motion
         pl bl tl)

    (let ((r (random (length (elt pose-list 0)))))
          (setq st-p (elt (elt pose-list 0) r))
          (setq st-b (elt (elt pose-list 0) r)))

    (let ((p st-p)
          next-p tmp-p min-dist dist p-idx)
      (dotimes (i (length pose-list)) ;;door angle
        (setq min-dist nil)
        (dotimes (j (length (elt pose-list i)))
          (setq tmp-p (elt (elt pose-list i) j))
          (setq dist (v- p tmp-p))
          (setf (elt dist 0) 0)
          (setq dist (norm dist))
          (when (or (not min-dist) (< dist min-dist))
            (setq min-dist dist)
            (setq p-idx j)))
          
        (if (elt (elt pose-list i) p-idx)
            (progn
              (push (elt (elt pose-list i) p-idx) pl)
              (push (elt (elt base-list i) p-idx) bl)
              (push (elt (elt torque-list i) p-idx) tl))
          (progn
            (push nil pl)
            (push nil bl)
            (push nil tl)))
      (setq p (elt (elt pose-list i) p-idx))))

    (setq motion (list (reverse pl) (reverse bl) (reverse tl)))
    motion))

(defun min-torque-motion 
  (&optional (arm :rarm) (idx 0))
  (let* ((pose-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :pose-seq) idx))
         (base-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :base-seq) idx))
         (torque-list (elt (send (if (eq arm :rarm) *filtered-right-samples* *filtered-left-samples*) :get :torque-seq) idx))
         st-p st-b
         motion
         pl bl tl)

    (let ((r (random (length (elt pose-list 0)))))
          (setq st-p (elt (elt pose-list 0) r))
          (setq st-b (elt (elt pose-list 0) r)))

    (let ((p st-p)
          next-p tmp-p min-torque tmp-t torque p-idx)
      (dotimes (i (length pose-list)) ;;door angle
        (setq min-torque nil)
        (dotimes (j (length (elt pose-list i)))
          (setq tmp-t (elt (elt torque-list i) j))
          (setf (elt tmp-t 0) 0)
          (setq torque (norm tmp-t))
          (when (or (not min-torque) (< torque min-torque))
            (setq min-torque torque)
            (setq p-idx j)))

        (if (elt (elt pose-list i) p-idx)
            (progn
              (push (elt (elt pose-list i) p-idx) pl)
              (push (elt (elt base-list i) p-idx) bl)
              (push (elt (elt torque-list i) p-idx) tl))
          (progn
            (push nil pl)
            (push nil bl)
            (push nil tl)))
      (setq p (elt (elt pose-list i) p-idx))))

    (setq motion (list (reverse pl) (reverse bl) (reverse tl)))
    motion))


    

    
    
