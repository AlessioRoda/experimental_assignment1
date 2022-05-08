(define (problem problem1) 
(:domain domain_sherlock)
(:objects 
    sherlock - robot
    w1 w2 w3 w4 oracle_room - waypoint
    

)

(:init
    
(in_position oracle_room sherlock)
;Notice that the distance described here are proportionally to the real distances, but haven't the purpose to provide a real value
;Provide all the possible distances, it's up to the robot to find the best planning 
(distance w1 w2 5)
(distance w2 w1 5)
(distance w1 w4 5)
(distance w4 w1 5)
(distance w2 w3 5)
(distance w2 w1 5)
(distance w3 w2 5)
(distance w3 w4 5)
(distance w1 oracle_room 2.5*1.41) ;distance between the corner of the room and the center
(distance w2 oracle_room 2.5*1.41)
(distance w3 oracle_room 2.5*1.41)
(distance w4 oracle_room 2.5*1.41)
(distance oracle_room w1 2.5*1.41)
(distance oracle_room w2 2.5*1.41)
(distance oracle_room w3 2.5*1.41)
(distance oracle_room w4 2.5*1.41)
(distance w1 w3 5*1.41) 
(distance w2 w4 5*1.41)
(distance w3 w1 5*1.41)
(distance w4 w2 5*1.41)

(not_can_check)
(ontology_updated)
(not_get_hint w1)
(not_get_hint w2)
(not_get_hint w3)
(not_get_hint w4)
(not_visited w1)
(not_visited w2)
(not_visited w3)
(not_visited w4)


)

(:goal 
    (and
    (visited w1)
    (visited w2)
    (visited w3)
    (visited w4)
    (end_game)

    )
)
