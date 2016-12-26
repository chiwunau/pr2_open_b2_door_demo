(setq *cof-p* 0.0)
(defun pose-diff-cost
  (p1 p2 b1 b2) ;;pose1(reccent) pose2(next) base1(reccent) base2(next)
  (let* ((cof-p *cof-p*)
         (cof-b (- 1 cof-p))
         (cof-b-trans 1.0)
         (cof-b-rot 1.0)
         (diff-p (v- p2 p1))
         (diff-cds (send (send b1 :transformation b2) :copy-worldcoords)))

    (setf (elt diff-p 0) 0)
    (setq diff-p (deg2rad (norm diff-p)))
    (setq diff-b (+ (* cof-b-trans (/ (norm (send diff-cds :pos)) 1000))
                    (* cof-b-rot (abs (car (car (send diff-cds :rpy-angle)))))))

    ;; (format a "~A~%" diff-p)
    ;; (format b "~A~%" diff-b)
    ;; (format c "~A~%" (/ (norm (send diff-cds :pos)) 1000))
    ;; (format d "~A~%" (abs (car (car (send diff-cds :rpy-angle)))))
    ;; (format t "~A, ~A, " diff-p diff-b)
    ;; (format t "~A, ~A~%" (/ (norm (send diff-cds :pos)) 1000) (abs (car (car (send diff-cds :rpy-angle)))))
    (+ (* cof-p diff-p) (* cof-b diff-b))))

(defun obstacle-dist-cost (&key node obstacle)
  (unless obstacle
    (return-from obstacle-dist-cost 0.0))
  (if (send node :get-value :robot-pose-optimized)
      (progn
        (send *pr2* :angle-vector (send node :get-value :robot-pose-optimized))
        (send *pr2* :move-to (send node :get-value :base-coords-optimized) :world))
    (progn
      (send *pr2* :angle-vector (send node :get-value :robot-pose))
      (send *pr2* :move-to (send node :get-value :base-coords) :world)))
  
  (let ((d (car (pqp-collision-distance (send *pr2* :base_footprint_lk) obstacle)))
        (d-thres 200.0) ;;20cm
        cost)
    (if (> d 0.0)
        (cond
         ((>= d d-thres)
          (setq cost 0.0))
         ((< d d-thres)
          (setq cost (* (/ (- d-thres d) d-thres) 1.0))))
      (let ((shortest-dist (* (car (find-if #'(lambda (x) x) (send-all (send obstacle :faces) :intersect-line (send *pr2* :pos) (send obstacle :centroid))))
                              (distance (send *pr2* :pos) (send obstacle :centroid)))))
        ;; (print shortest-dist)
        (setq cost (* 1.0 (- (/ 480 shortest-dist) 1)))))
    cost))

(defun torque-limit-cost (&key node)
  (let* ((tq (if (send node :get-value :robot-pose-optimized) (send node :get-value :torque-optimized) (send node :get-value :torque)))
        (max-tq (send *pr2* :max-torque-vector))
        (tq-ratio-list (mapcar #'(lambda (tqi maxi) (abs (/ tqi maxi))) (coerce tq cons) (coerce max-tq cons)))
        (max-ratio (reduce 'max tq-ratio-list))
        (ratio-thres 0.6)
        cost)
    
    ;; (print tq-ratio-list)
    (cond 
     ((<= max-ratio ratio-thres)
      (setq cost 0.0))
     ((and (> max-ratio ratio-thres) (<= max-ratio 1.0))
      (setq cost (/ (- max-ratio ratio-thres) (- 1.0 ratio-thres))))
     ((> max-ratio 1.0)
      (setq cost (exp max-ratio))))
    cost))
      
      
      




        


;; (defun overload-node? (n &optional (door-weight 2.5))
;;   (let* ((force-cds (elt (send n :get :cds-info) 0))
;;          ;;             (contact-pt (elt (send n :get :cds-info) 2))
;;          (contact-pt (elt (reverse (set-contact-points (send n :get :arm))) (send n :get :contact-point)))
;;          (max-tq (send *pr2* :max-torque-vector))
;;          (overload nil)
;;          drw tq)
;;     (send *pr2* :angle-vector (send n :get :pose))
;;     (send *pr2* :move-to (send n :get :base) :world)
;;     (send *door* :hinge :joint-angle (send n :get :door-angle))
    
;;     (setq drw (door-reaction-wrench force-cds contact-pt door-weight))
;;     (setq tq (send *pr2* :calc-torque-from-ext-wrenches :force-list (list (car drw)) :moment-list (list (cadr drw)) :target-coords (list contact-pt)))
    
;;     (dotimes (i (length max-tq))
;;       (if (> 0 (- (abs (elt max-tq i)) (abs (elt tq i))))
;;           (setq overload t)))

;;     overload))

(defun colide-obstacle? (n)
  (unless *obs*
    (return-from colide-obstacle? nil))
  (if (send n :get-value :robot-pose-optimized)
      (progn
        (send *pr2* :angle-vector (send n :get-value :robot-pose-optimized))
        (send *pr2* :move-to (send n :get-value :base-coords-optimized) :world))
    (progn
      (send *pr2* :angle-vector (send n :get-value :robot-pose))
      (send *pr2* :move-to (send n :get-value :base-coords) :world)))
  (pqp-collision-check-objects 
   (list (send *pr2* :base_footprint_lk))
   (if (listp *obs*)
       *obs*
     (list *obs*)))
  )

  
  


;; (defun transitable? (n1 n2)
;;   (let* ((mid-pose (cadr (cadr (pos-list-interpolation (list (send n1 :get-value :pose) (send n2 :get-value :pose)) (list 1000) 500))))
;;          (diff-cds (send (send (send n1 :get-value :base) :transformation (send n2 :get-value :base)) :copy-worldcoords))
;;          (mid-cds (send (send (send (send n1 :get-value :base) :copy-worldcoords) :translate (scale 0.5 (send diff-cds :pos))) :rotate (* 0.5 (caar (send diff-cds :rpy-angle))) :z))
;;          (mid-door-angle (/ (+ (send n1 :get-value :door-angle) (send n2 :get-value :door-angle)) 2)))

;;     (send *pr2* :angle-vector mid-pose)
;;     (send *pr2* :move-to mid-cds :world)
;;     (send *door* :hinge :joint-angle mid-door-angle)
;;     t
;;    (not (pqp-collision-check-objects (append (send *pr2* :torso :parent-link) (send *pr2* (send n1 :get-value :arm) :links))  (send *door* :links)))
;;     ))

(defun play-middle-state (n1 n2 a b)
  (let* ((pose-pli (pos-list-interpolation (list (if (send n1 :get-value :robot-pose-optimized)
                                                     (send n1 :get-value :robot-pose-optimized)
                                                   (send n1 :get-value :robot-pose))
                                                 (if (send n2 :get-value :robot-pose-optimized)
                                                     (send n2 :get-value :robot-pose-optimized)
                                                   (send n2 :get-value :robot-pose)))
                                           (list (* (- b 1) 250)) 250))
         (diff-cds (send (send (if (send n1 :get-value :robot-pose-optimized)
                                   (send n1 :get-value :base-coords-optimized)
                                 (send n1 :get-value :base-coords))
                               :transformation (if (send n2 :get-value :robot-pose-optimized)
                                                   (send n2 :get-value :base-coords-optimized)
                                                 (send n2 :get-value :base-coords)))
                         :copy-worldcoords))
         (base-pos-rot-v (coerce (append (coerce (send diff-cds :pos) cons) (list (caar (send diff-cds :rpy-angle)))) float-vector))
         (base-pli (pos-list-interpolation (list (coerce (make-list (length base-pos-rot-v) :initial-element 0) float-vector) base-pos-rot-v) (list (* (- b 1) 250)) 250))
         (mid-cds (send (send (send (if (send n1 :get-value :robot-pose-optimized)
                                        (send n1 :get-value :base-coords-optimized)
                                      (send n1 :get-value :base-coords)) :copy-worldcoords)
                              :translate (subseq (elt (cadr base-pli) a) 0 3))
                        :rotate (elt (elt (cadr base-pli) a) 3) :z))
         )
    
    ;; (setq *pose-pli* pose-pli)
    ;; (setq *mid-cds* mid-cds)
    (send *door* :hinge :joint-angle (+ (send n1 :get-value :door-angle) (* a (/ (- (send n2 :get-value :door-angle) (send n1 :get-value :door-angle)) (- b 1)))))
    (send *pr2* :angle-vector (elt (cadr pose-pli) a))
    (send *pr2* :move-to mid-cds :world)
    (send *irtviewer* :draw-objects)

    )
  )



(defun transitable? (n1 n2)
(return-from transitable? t) ;;for-debug
  (let* ((pose-pli (pos-list-interpolation (list (send n1 :get-value :pose) (send n2 :get-value :pose)) (list 750) 250))
         (diff-cds (send (send (send n1 :get-value :base) :transformation (send n2 :get-value :base)) :copy-worldcoords))
         (base-pos-rot-v (coerce (append (coerce (send diff-cds :pos) cons) (list (caar (send diff-cds :rpy-angle)))) float-vector))
         (base-pli (pos-list-interpolation (list (coerce (make-list (length base-pos-rot-v) :initial-element 0) float-vector) base-pos-rot-v) (list 750) 250)))
;;         (mid-cds (send (send (send (send n1 :get-value :base) :copy-worldcoords) :translate (scale 0.5 (send diff-cds :pos))) :rotate (* 0.5 (caar (send diff-cds :rpy-angle))) :z))
  ;;       (mid-door-angle (/ (+ (send n1 :get-value :door-angle) (send n2 :get-value :door-angle)) 2)))

    (dotimes (i (length (cadr pose-pli)))
      (let ((mid-cds (send (send (send (send n1 :get-value :base) :copy-worldcoords)
                                 :translate (subseq (elt (cadr base-pli) i) 0 3))
                           :rotate (elt (elt (cadr base-pli) i) 3) :z))
            (mid-door-angle (+ (send n1 :get-value :door-angle) (* i (/ (- (send n2 :get-value :door-angle) (send n1 :get-value :door-angle)) (* 1.0 (- (length (cadr pose-pli)) 1)))))))
        (send *pr2* :angle-vector (elt (cadr pose-pli) i))
        (send *pr2* :move-to mid-cds :world)
        (send *door* :hinge :joint-angle (+ mid-door-angle -3))
        ;; (print (elt (cadr pose-pli) i))
        ;; (print mid-cds)
        ;; (print mid-door-angle)
     ;; (print mid-door-angle)
     ;; (send *irtviewer* :draw-objects)
     ;; (unix::usleep (* 1000 500))
        (let (tmpl) 
          (dolist (dl (send *door* :links)) (unless (string-equal (send dl :name) "rear-knob")
                                                (push dl tmpl)))
          (when (or (pqp-collision-check-objects (append (send *pr2* :torso :parent-link) (send *pr2* (send n1 :get-value :arm) :links)) (send *door* :links)) (check-arm-self-collision (send n1 :get-value :arm)))
            ;; (send *irtviewer* :draw-objects)
            ;; (unix::usleep (* 1000 500))
            (return-from transitable? nil))
          (dotimes (i 90) ;;make sure robot's arm do not collide with rear knob
            (when (pqp-collision-check-objects (send *pr2* (send n1 :get-value :arm)  :links) (list (elt (send *door* :links) (position "rear-knob" (send-all (send *door* :links) :name) :test 'string-equal))))
              (return-from transitable? nil))
            (send *door* :hinge :joint-angle (+ i -90))
;;            (send *irtviewer* :draw-objects)
            )
        )))
    t))

(defun check-arm-self-collision
  (&optional (arm :larm))
  (let (tmpl)
    (dolist (l (send *pr2* :links)) (unless (member l (send *pr2* arm :links)) (push l tmpl)))
    (push (car (send *pr2* :torso :parent-link)) tmpl)
    (pqp-collision-check-objects (cdr (send *pr2* arm :links)) tmpl)))

(defun get-node (name)
  (let ((p (position name (send-all *all-nodes* :name) :test 'equal)))
    (if p
        (elt *all-nodes* p)
      (progn
        (setq p (position name (send-all *bridge-nodes* :name) :test 'equal))
        (if p
            (elt *bridge-nodes* p))))))


(defun refine-postures-in-path (&optional (p *path*))
  (set-spare-arm-tuck-arm-pose p)
  (refine-path-continuous-joint-ang p)
  (prevent-wrist-rotate p))


(defun set-spare-arm-tuck-arm-pose (&optional (p *path*))
  (let ((ns (send-all p :state)))
    (dolist (n ns)
      (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
        (unless (send n :get-value :islands)
          (if (send n :get-value :robot-pose-optimized)
            (let ((spare-arm (if (eq (send n :get-value :arm) :rarm) :larm :rarm)))
              (send *pr2* :angle-vector (send n :get-value :robot-pose-optimized))
              (send *pr2* spare-arm :angle-vector (get-tuckarm (if (eq spare-arm :rarm) :larm :rarm) :outside spare-arm))
              (send *pr2* spare-arm :shoulder-r :joint-angle (if (eq spare-arm :rarm) -100 100))
              (send n :put-value :robot-pose-optimized (send *pr2* :angle-vector))))
          (let ((spare-arm (if (eq (send n :get-value :arm) :rarm) :larm :rarm)))
            (send *pr2* :angle-vector (send n :get-value :robot-pose))
            (send *pr2* spare-arm :angle-vector (get-tuckarm (if (eq spare-arm :rarm) :larm :rarm) :outside spare-arm))
            (send *pr2* spare-arm :shoulder-r :joint-angle (if (eq spare-arm :rarm) -100 100))
            (send n :put-value :robot-pose (send *pr2* :angle-vector))))))))
            

(defun refine-path-continuous-joint-ang (&optional (p *path*))
  (let ((ns (send-all p :state)))
    (dolist (n ns)
      (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
        (if (send n :get-value :robot-pose-optimized)
            (progn 
              (send *pr2* :angle-vector (send n :get-value :robot-pose-optimized))
              (send *pr2* :larm :elbow-r :joint-angle (refine-continuous-joint-ang (send *pr2* :larm :elbow-r :joint-angle) 360))
              (send *pr2* :larm :wrist-r :joint-angle (refine-continuous-joint-ang (send *pr2* :larm :wrist-r :joint-angle) 180))
              (send *pr2* :rarm :elbow-r :joint-angle (refine-continuous-joint-ang (send *pr2* :rarm :elbow-r :joint-angle) 360))
              (send *pr2* :rarm :wrist-r :joint-angle (refine-continuous-joint-ang (send *pr2* :rarm :wrist-r :joint-angle) 180))
              (send n :put-value :robot-pose-optimized (send *pr2* :angle-vector))))
        
        (send *pr2* :angle-vector (send n :get-value :robot-pose))
        (send *pr2* :larm :elbow-r :joint-angle (refine-continuous-joint-ang (send *pr2* :larm :elbow-r :joint-angle) 360))
        (send *pr2* :larm :wrist-r :joint-angle (refine-continuous-joint-ang (send *pr2* :larm :wrist-r :joint-angle) 180))
        (send *pr2* :rarm :elbow-r :joint-angle (refine-continuous-joint-ang (send *pr2* :rarm :elbow-r :joint-angle) 360))
        (send *pr2* :rarm :wrist-r :joint-angle (refine-continuous-joint-ang (send *pr2* :rarm :wrist-r :joint-angle) 180))
        (send n :put-value :robot-pose (send *pr2* :angle-vector))))))


(defun refine-continuous-joint-ang (ang repeat-ang)
  (cond ((> (mod ang repeat-ang) (/ repeat-ang 2))
         (setq ang (- (mod ang repeat-ang) repeat-ang)))
        ((>= (mod ang repeat-ang) 0.0)
         (setq ang (mod ang repeat-ang)))

        ((< (mod ang repeat-ang) (/ repeat-ang -2))
         (setq ang (+ (mod ang repeat-ang) repeat-ang)))
        ((<= (mod ang repeat-ang) 0.0)
         (setq ang (mod ang repeat-ang))))
  ang)

(defun prevent-wrist-rotate (&optional (p *path*))
  (let ((ns (send-all p :state)))
    (do ((i 0 (+ i 1))) ((>= i (length ns)))
        (let ((n (elt ns i))
              (next-n (if (< i (- (length ns) 1)) (elt ns (+ i 1))))
              (pre-n (if (> i 0) (elt ns (- i 1)))))
          (unless (or (string-equal (send n :name) 'start) (string-equal (send n :name) 'goal))
            (play-node-best n :draw-objects nil)
            (same-end-coords-force-elbow-wrist-counter-rotate pre-n n)
            (if (send n :get-value :robot-pose-optimized)
                (send n :put-value :robot-pose-optimized (send *pr2* :angle-vector))
              (send n :put-value :robot-pose (send *pr2* :angle-vector))))))))

          

          

          
(defun same-end-coords-force-elbow-wrist-counter-rotate (pre-n n)
  (cond ((and (send pre-n :get-value :islands) (eq (send n :get-value :contact-type) 'grasp))
         (let (ep1 ep2
                   elbow-r1 elbow-r2
                   wrist-r1 wrist-r2)
           (play-node-best pre-n :draw-objects nil)
           (setq ep1 (send *pr2* (send n :get-value :arm) :end-coords :worldpos))
           (setq elbow-r1 (send *pr2* (send n :get-value :arm) :elbow-r :joint-angle))
           (setq wrist-r1 (send *pr2* (send n :get-value :arm) :wrist-r :joint-angle))
           (play-node-best n :draw-objects nil)
           (setq ep2 (send *pr2* (send n :get-value :arm) :end-coords :worldpos))
           (setq elbow-r2 (send *pr2* (send n :get-value :arm) :elbow-r :joint-angle))
           (setq wrist-r2 (send *pr2* (send n :get-value :arm) :wrist-r :joint-angle))
           
           ;; (if (and (< (norm (v- ep1 ep2)) 5) (or (and (> (- elbow-r2 elbow-r1) 0) (> (- wrist-r2 wrist-r1) 0))
           ;;                                        (and (< (- elbow-r2 elbow-r1) 0) (< (- wrist-r2 wrist-r1) 0))))
           (if (> (abs (- wrist-r2 wrist-r1)) 90)
               (if (< wrist-r2 0)
                   (send *pr2* (send n :get-value :arm) :wrist-r :joint-angle (+ wrist-r2 180))
                 (send *pr2* (send n :get-value :arm) :wrist-r :joint-angle (- wrist-r2 180))))))
         

        ((and (send n :get-value :islands) (eq (send pre-n :get-value :contact-type) 'grasp))
         (let (ep1 ep2
                   elbow-r1 elbow-r2
                   wrist-r1 wrist-r2)
           (play-node-best pre-n :draw-objects nil)
           (setq ep1 (send *pr2* (send pre-n :get-value :arm) :end-coords :worldpos))
           (setq elbow-r1 (send *pr2* (send pre-n :get-value :arm) :elbow-r :joint-angle))
           (setq wrist-r1 (send *pr2* (send pre-n :get-value :arm) :wrist-r :joint-angle))
           (play-node-best n :draw-objects nil)
           (setq ep2 (send *pr2* (send pre-n :get-value :arm) :end-coords :worldpos))
           (setq elbow-r2 (send *pr2* (send pre-n :get-value :arm) :elbow-r :joint-angle))
           (setq wrist-r2 (send *pr2* (send pre-n :get-value :arm) :wrist-r :joint-angle))

           ;; (if (and (< (norm (v- ep1 ep2)) 5) (or (and (> (- elbow-r2 elbow-r1) 0) (> (- wrist-r2 wrist-r1) 0))
           ;;                                        (and (< (- elbow-r2 elbow-r1) 0) (< (- wrist-r2 wrist-r1) 0))))
           (if (> (abs (- wrist-r2 wrist-r1)) 90)
                   (if (< wrist-r2 0)
                       (send *pr2* (send pre-n :get-value :arm) :wrist-r :joint-angle (+ wrist-r2 180))
                     (send *pr2* (send pre-n :get-value :arm) :wrist-r :joint-angle (- wrist-r2 180))))))
         

        ((and (eq (send n :get-value :contact-type) 'grasp) (eq (send pre-n :get-value :contact-type) 'grasp))
         (let (ep1 ep2
                   elbow-r1 elbow-r2
                   wrist-r1 wrist-r2)
           (play-node-best pre-n :draw-objects nil)
           (setq ep1 (send *pr2* (send pre-n :get-value :arm) :end-coords :worldpos))
           (setq elbow-r1 (send *pr2* (send pre-n :get-value :arm) :elbow-r :joint-angle))
           (setq wrist-r1 (send *pr2* (send pre-n :get-value :arm) :wrist-r :joint-angle))
           (play-node-best n :draw-objects nil)
           (setq ep2 (send *pr2* (send n :get-value :arm) :end-coords :worldpos))
           (setq elbow-r2 (send *pr2* (send n :get-value :arm) :elbow-r :joint-angle))
           (setq wrist-r2 (send *pr2* (send n :get-value :arm) :wrist-r :joint-angle))
           
           ;; (if (and (< (norm (v- ep1 ep2)) 5) (or (and (> (- elbow-r2 elbow-r1) 0) (> (- wrist-r2 wrist-r1) 0))
           ;;                                        (and (< (- elbow-r2 elbow-r1) 0) (< (- wrist-r2 wrist-r1) 0))))
               (if (> (abs (- wrist-r2 wrist-r1)) 90)
                   (if (< wrist-r2 0)
                       (send *pr2* (send n :get-value :arm) :wrist-r :joint-angle (+ wrist-r2 180))
                     (send *pr2* (send n :get-value :arm) :wrist-r :joint-angle (- wrist-r2 180))))))
        )
  
)
