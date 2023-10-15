(define (problem example_problem)
(:domain tamp)
(:objects
        wp0 wp1 wp2 wp3 - normal_way
        robot0000 robot0001 - robot
        goal0000 goal0001 - job
        bolt weld - single_task
)
(:init
        (connected wp0 wp1)
        (= (distance wp0 wp1) 3.08)
        (connected wp0 wp2)
        (= (distance wp0 wp2) 18.94)
        (connected wp0 wp3)
        (= (distance wp0 wp3) 9.12)

        (connected wp1 wp0)
        (= (distance wp1 wp0) 3.08)
        (connected wp1 wp2)
        (= (distance wp1 wp2) 7.1)

        (vertex_free wp2)
        (connected wp2 wp0)
        (= (distance wp2 wp0) 18.94)
        (connected wp2 wp1)
        (= (distance wp2 wp1) 7.1)

        (vertex_free wp3)
        (connected wp3 wp0)
        (= (distance wp3 wp0) 9.12)

        (at robot0000 wp0)
        (= (velocity robot0000) 1.0)
        (= (task_duration robot0000 bolt) 5)
        (can_perform robot0000 bolt)
        (= (task_duration robot0000 weld) 9)
        (can_perform robot0000 weld)

        (at robot0001 wp1)
        (= (velocity robot0001) 1.0)
        (= (task_duration robot0001 weld) 5)
        (can_perform robot0001 weld)
        (= (task_duration robot0001 bolt) 9)
        (can_perform robot0001 bolt)

        (located goal0000 wp2)
        (work_free goal0000)

        (located goal0001 wp3)
        (work_free goal0001)

        (isJobOfType goal0000 bolt)

        (isJobOfType goal0001 weld)

)
(:goal (and
        (finish goal0000 bolt)
        (finish goal0001 weld)
        )
)
(:metric minimize (total-time))
)
