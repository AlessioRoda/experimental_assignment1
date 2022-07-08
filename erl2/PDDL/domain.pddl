(define (domain domain_sherlock)

    (:requirements :strips 
                    :typing
                    :durative-actions 
                    :fluents
    )

    (:types
        robot waypoint
    )

    (:functions
        (waypoints)
        (cost)
        (distance ?from ?to - waypoint)
    )

    (:predicates
            (in_position ?p - waypoint ?r - robot)
            (visited ?w - waypoint)
            (not_visited ?w - waypoint)
            (get_hint ?w - waypoint)
            (not_get_hint ?w - waypoint)
            (not_ontology_updated)
            (can_check)
            (not_can_check)
            (end_game)

    )



    (:durative-action go_to_waypoint
        :parameters (?from ?to - waypoint ?r - robot)
        :duration (= ?duration (distance ?from ?to))
        :condition (and 
            (at start (and (in_position ?from ?r)(not_visited ?to)
            ))
        )
        :effect (and 
            (at end (and (in_position ?to ?r)(visited ?to)(not(in_position ?from ?r))(not(not_visited ?to))(increase (cost) (distance ?from ?to))
            ))
        )
    )

    (:durative-action movearm
        :parameters (?r - robot ?w - waypoint)
        :duration (= ?duration 1)
        :condition (and 
            (at start (and (not_get_hint ?w)(in_position ?w ?r)
            ))
        )
        :effect (and 
            (at end (and (not (not_get_hint ?w))(get_hint ?w)(increase (waypoints) 1)(increase (cost) 1)
            ))
        )
    )

    (:durative-action update_ontology
        :parameters ()
        :duration (= ?duration 1)
        :condition (and 
            (at start (and (>= (waypoints) 4)
            ))
        )
        :effect (and 
            (at end (and (can_check)(increase (cost) 1)
            ))
        )
    )
    
    (:durative-action check_consistency
        :parameters ()
        :duration (= ?duration 1)
        :condition (and 
            (at start (and (can_check)
            ))
        )
        :effect (and 
            (at end (and (end_game)(increase (cost) 1)
            ))
        )
    )
    
)