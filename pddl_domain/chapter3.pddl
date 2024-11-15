(define (domain tamp)

(:requirements :strips :fluents :durative-actions :typing :conditional-effects :adl :continuous-effects :duration-inequalities :universal-preconditions)

(:types waypoint robot job jobtype - object
        normal_way col_way - waypoint
        single_task double_task - jobtype
)

(:predicates (at ?r - robot ?w - waypoint)
             (connected ?w1 ?w2 - waypoint)
             (located ?j - job ?w - waypoint)
             (vertex_free ?w - waypoint)
             (work_free ?j - job)
             (finish ?j - job ?t - jobtype)
             (isJobOfType ?j - job ?t - jobtype)
             (can_perform ?r - robot ?t - jobtype)
             (robot_free ?r - robot)
             (before ?j - job ?t1 ?t2 - jobtype)
)

(:functions (distance ?from ?to - waypoint)
            (task_duration ?r - robot ?t - jobtype)
            (velocity ?r - robot)
)


(:durative-action normal_navigate
    :parameters (?r - robot ?from ?to - normal_way)
    :duration (= ?duration (/ (distance ?from ?to) (velocity ?r)))
    :condition (and (at start (at ?r ?from))
                    (over all (vertex_free ?to))
                    ;(at end (vertex_free ?to))
                    (over all (connected ?from ?to)))
    :effect (and (at start (not (at ?r ?from)))
                 (at start (vertex_free ?from))
                 (at end (not (vertex_free ?to)))
                 (at end (at ?r ?to)))
)

(:durative-action do_task_single
    :parameters (?r - robot ?w - waypoint ?j - job ?s - single_task)
    :duration (= ?duration (task_duration ?r ?s))
    :condition (and (at start (work_free ?j))
    		     (at start (robot_free ?r))
                    (at start (isJobOfType ?j ?s))
                    (at start (can_perform ?r ?s))
                    (over all (at ?r ?w))
                    (over all (located ?j ?w))
                    (at start (forall (?x - jobtype) (imply (before ?j ?x ?s) (finish ?j ?x)))))
    :effect (and (at start (not (work_free ?j)))
                 (at start (not (robot_free ?r)))
                 (at end (finish ?j ?s))
                 (at end (robot_free ?r))
                 (at end (work_free ?j)))
)


)
