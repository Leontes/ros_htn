(define (problem lastPlan) (:domain Rover)
	(:objects
		general - lander
		rover0 - rover
		rover0store - store
		waypoint0 - waypoint
		waypoint1 - waypoint
		waypoint2 - waypoint
		camera0 - camera
		objective0 - objective
	)
	(:init
		(at_soil_sample waypoint1)
		(at_rock_sample waypoint2)
		(visible_from objective0 waypoint2)
		(calibration_target camera0 objective0)
		(in_sun waypoint2)
		(at_lander general waypoint0)
		(channel_free general)
		(visible waypoint0 waypoint1)
		(visible waypoint0 waypoint2)
		(store_of rover0store rover0)
		(empty rover0store)
		(equipped_for_rock_analysis rover0)
		(equipped_for_soil_analysis rover0)
		(equipped_for_imaging rover0)
		(visible waypoint1 waypoint2)
		(on_board camera0 rover0)
		(supports camera0 colour)
		(supports camera0 high_res)
		(supports camera0 low_res)
		(= (energy rover0) 5000)
		(= (recharge-rate rover0) 5)
		(at rover0 waypoint0)
		(available rover0)
		(can_traverse rover0 waypoint0 waypoint1)
		(can_traverse rover0 waypoint0 waypoint2)
		(can_traverse rover0 waypoint1 waypoint2)
	)
	(:tasks-goal
		:tasks(
			(sample rover0 waypoint1)
			(sample rover0 waypoint2)
			(photo rover0 objective0 high_res)
		)
	)
)
