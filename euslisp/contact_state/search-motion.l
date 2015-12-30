(setq *sampled-motion-sequence* nil)


(defun motion-generation ()
  (let ((tgt-cds-list (send *door* :get-front-touch-cds-list))
        motion-list
        res)
    (dolist (cds tgt-cds-list)
      (setq res (send *pr2* :larm :inverse-kinematics cds :rotation-axis nil :debug-view :no-message))
      (push res motion-list))
  (push motion-list *sampled-motion-sequence*)))
   
   
    
