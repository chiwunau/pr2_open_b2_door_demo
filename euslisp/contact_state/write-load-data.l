(defun update-loaded-nodes (&key (robot *pr2*) (door *door*) (nodes *all-nodes*))
  (let ((r-contact-pts (set-contact-points :rarm :robot robot))
        (l-contact-pts (set-contact-points :larm :robot robot))
        (obj-cds-list (send door :get-rear-touch-cds-list)))
    (dolist (n nodes)
      (send n :put-value :contact-point (elt (if (eq (send n :get-value :arm) :rarm) r-contact-pts l-contact-pts) (send n :get-value :contact-point-index)))
      (case (send n :get-value :object-coords-index)
            ('fk (send n :put-value :object-coords (send door :front-knob-coords)))
            ('rk (send n :put-value :object-coords (send door :rear-knob-coords)))
            (t (send n :put-value :object-coords (elt obj-cds-list (send n :get-value :object-coords-index)))))

      (if (numberp (send n :get-value :object-coords-index))
          (send n :put-value :ik-target-coords (send (send n :get-value :object-coords) :assoc (make-cascoords :coords (send (send (send (send n :get-value :object-coords) :copy-worldcoords) :translate *ik-target-shift*) :rotate (if (eq (send n :get-value :contact-point-index) 0) 0 pi/2) :y))))
        (send n :put-value :ik-target-coords (send (send n :get-value :object-coords) :assoc (make-cascoords :coords (send (send (send (send (send n :get-value :object-coords) :copy-worldcoords) :rotate (if (eq (send n :get-value :object-coords-index) 'fk) 0 pi) :z) :rotate pi/2 :x) :translate #f(15 0 0)))))
        ))))


(defun save-data (path)
  (dolist (n *all-nodes*)
    (send n :put-value :contact-point nil)
    (send n :put-value :object-coords nil)
    (send n :put-value :ik-target-coords nil))

  (dump-loadable-structure path *all-nodes* *bridge-nodes*))
