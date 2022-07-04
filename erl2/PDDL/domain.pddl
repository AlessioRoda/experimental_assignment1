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
    )

    (:predicates
            (in_position ?p - waypoint ?r - robot)
            (waypoint_pose ?w - waypoint)
            (visited ?w - waypoint)
            (not_visited ?w - waypoint)
            (get_hint ?w - waypoint)
            (not_get_hint ?w - waypoint)
            (ontology_updated)
            (not_ontology_updated)
            (distance ?from ?to - waypoint)
            (can_check)
            (not_can_check)
            (end_game)

    )
;(= (?duration) (distance ?from ?to))
    (:durative-action go_to_waypoint
        :parameters (?from ?to - waypoint ?r - robot)
        :duration (= ?duration 5)
        :condition (and 
            (at start (and (in_position ?from ?r)(waypoint_pose ?to)(not_visited ?to)
            ))
        )
        :effect (and 
            (at end (and (increase (waypoints) 1) (in_position ?to ?r)(visited ?to)(not(in_position ?from ?r)(not_visited ?to))
            ))
        )
    )

    (:durative-action movearm
        :parameters (?r - robot ?w - waypoint)
        :duration (= ?duration 1)
        :condition (and 
            (at start (and (not_get_hint ?w)(in_position ?r ?w)(ontology_updated)
            ))
        )
        :effect (and 
            (at end (and (not (not_get_hint ?w)(ontology_updated))(get_hint ?w)(not_ontology_updated)
            ))
        )
    )

    (:durative-action updateOntology
        :parameters ()
        :duration (= ?duration 1)
        :condition (and 
            (at start (and (=(waypoints)4)(not_ontology_updated)(not_can_check)
            ))
        )
        :effect (and 
            (at end (and (assign (waypoints) 0)(ontology_updated)(can_check)(not(not_ontology_updated)(not_can_check))
            ))
        )
    )
    
    (:durative-action checkConsistency
        :parameters ()
        :duration (= ?duration 1)
        :condition (and 
            (at start (and (can_check)
            ))
        )
        :effect (and 
            (at end (and (end_game) 
            ))
        )
    )
    
)