(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 - region
     R2D2 - robot
)
(:init
    (robot_in R2D2 r0)
    (localized)			;;remove it when running dom1.txt code also frm goal

    (= (act-cost) 0)
    (= (dummy) 0)
 
)
(:goal 
     (and (visited r1)  (visited r2)
          (visited r3) (visited r4) (localized))
)
(:metric minimize (act-cost) )
)


