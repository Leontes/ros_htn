(define (domain Rover)
  (:requirements :typing :fluents :negative-preconditions :durative-actions :derived-predicates :htn-expansion)
  (:types rover waypoint store camera mode lander objective)
  (:constants colour high_res low_res - mode)

  (:predicates
    (at ?x - rover ?y - waypoint)
    (at_lander ?x - lander ?y - waypoint)
    (can_traverse ?r - rover ?x - waypoint ?y - waypoint)
  	(equipped_for_soil_analysis ?r - rover)
    (equipped_for_rock_analysis ?r - rover)
    (equipped_for_imaging ?r - rover)
    (empty ?s - store)
    (have_rock_analysis ?r - rover ?w - waypoint)
    (have_soil_analysis ?r - rover ?w - waypoint)
    (full ?s - store)
  	(calibrated ?c - camera ?r - rover)
  	(supports ?c - camera ?m - mode)
    (available ?r - rover)
    (visible ?w - waypoint ?p - waypoint)
    (have_image ?r - rover ?o - objective ?m - mode)
    (communicated_soil_data ?w - waypoint)
    (communicated_rock_data ?w - waypoint)
    (communicated_image_data ?o - objective ?m - mode)
  	(at_soil_sample ?w - waypoint)
  	(at_rock_sample ?w - waypoint)
    (visible_from ?o - objective ?w - waypoint)
  	(store_of ?s - store ?r - rover)
  	(calibration_target ?i - camera ?o - objective)
  	(on_board ?i - camera ?r - rover)
  	(channel_free ?l - lander)
  	(in_sun ?w - waypoint)
    (equal ?x ?x)
    (different ?x ?y)
    (no-battery ?rov - rover)
    (can_traverseBI ?rov - rover ?wp1 - waypoint ?wp2 - waypoint)
    (can-see ?wp1 - waypoint ?wp2 - waypoint)
  )

  (:functions
    (energy ?r - rover)
    (recharge-rate ?r - rover)
  )

  ;;Predicados derivados

  (:derived
    (can_traverseBI ?rov - rover ?wp1 - waypoint ?wp2 - waypoint)
    (or (can_traverse ?rov ?wp1 ?wp2) (can_traverse ?rov ?wp2 ?wp1))
  )

  (:derived
    (can-see ?wp1 - waypoint ?wp2 - waypoint)
    (or (visible ?wp1 ?wp2) (visible ?wp2 ?wp1))
  )



  ;;Tareas Primitivas
  (:durative-action navigate
    :parameters (?x - rover ?y - waypoint ?z - waypoint)
    :duration (= ?duration 5)
    :condition
      (and
        (over all (can_traverseBI ?x ?y ?z))
        (at start (available ?x))
        (at start (at ?x ?y))
        (at start (>= (energy ?x) 8))
        (over all (can-see ?y ?z))
  	  )
    :effect
      (and
        (at start (decrease (energy ?x) 8))
        (at start (not (at ?x ?y)))
        (at end (at ?x ?z))
      )
  )

  (:durative-action recharge
    :parameters (?x - rover ?w - waypoint)
    :duration
      (= ?duration
        (/
          (- 80 (energy ?x))
          (recharge-rate ?x)
        )
      )
    :condition
      (and
        (at start (at ?x ?w))
        (over all (at ?x ?w))
  			(at start (in_sun ?w))
        (at start (<= (energy ?x) 80))
      )
    :effect
      (and
        (at end
          (increase (energy ?x)
            (* ?duration (recharge-rate ?x))
          )
        )
      )
  )

  (:durative-action sample_soil
    :parameters (?x - rover ?s - store ?p - waypoint)
    :duration (= ?duration 10)
    :condition
      (and
        (over all (at ?x ?p))
        (at start (at ?x ?p))
        (at start (at_soil_sample ?p))
        (at start (equipped_for_soil_analysis ?x))
        (at start (>= (energy ?x) 3))
        (at start (store_of ?s ?x))
        (at start (empty ?s))
  		)
    :effect
      (and
        (at start (not (empty ?s)))
        (at end (full ?s))
        (at start (decrease (energy ?x) 3))
        (at end (have_soil_analysis ?x ?p))
        (at end (not (at_soil_sample ?p)))
  		)
  )

  (:durative-action sample_rock
    :parameters (?x - rover ?s - store ?p - waypoint)
    :duration (= ?duration 8)
    :condition
      (and
        (over all (at ?x ?p))
        (at start (at ?x ?p))
        (at start (>= (energy ?x) 5))
        (at start (at_rock_sample ?p))
        (at start (equipped_for_rock_analysis ?x))
        (at start (store_of ?s ?x))
        (at start (empty ?s))
  		)
    :effect
      (and
        (at start (not (empty ?s)))
        (at end (full ?s))
        (at end (have_rock_analysis ?x ?p))
        (at start (decrease (energy ?x) 5))
        (at end (not (at_rock_sample ?p)))
  		)
  )

  (:durative-action drop
    :parameters (?x - rover ?y - store)
    :duration (= ?duration 1)
    :condition
      (and
        (at start (store_of ?y ?x))
        (at start (full ?y))
  		)
    :effect
      (and
        (at end (not (full ?y)))
        (at end (empty ?y))
      )
  )

  (:durative-action calibrate
    :parameters (?r - rover ?i - camera ?t - objective ?w - waypoint)
    :duration (= ?duration 5)
    :condition
      (and
        (at start (equipped_for_imaging ?r))
        (at start (>= (energy ?r) 2))
        (at start (calibration_target ?i ?t))
        (over all (at ?r ?w))
        (at start (visible_from ?t ?w))
        (at start (on_board ?i ?r))
  		)
    :effect
      (and
        (at end (calibrated ?i ?r))
        (at start (decrease (energy ?r) 2)))
  )

  (:durative-action take_image
    :parameters (?r - rover ?p - waypoint ?o - objective ?i - camera ?m - mode)
    :duration (= ?duration 7)
    :condition
      (and
        (over all (calibrated ?i ?r))
        (at start (on_board ?i ?r))
        (over all (equipped_for_imaging ?r))
        (over all (supports ?i ?m) )
        (over all (visible_from ?o ?p))
        (over all (at ?r ?p))
        (at start (>= (energy ?r) 1))
      )
    :effect
      (and
        (at end (have_image ?r ?o ?m))
        (at start (decrease (energy ?r) 1))
        (at end (not (calibrated ?i ?r)))
  		)
  )

  (:durative-action communicate_soil_data
    :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
    :duration (= ?duration 10)
    :condition
      (and
        (over all (at ?r ?x))
        (over all (at_lander ?l ?y))
        (at start (have_soil_analysis ?r ?p))
        (at start (>= (energy ?r) 4))
        (at start (can-see ?x ?y))
        (at start (available ?r))
        (at start (channel_free ?l))
      )
    :effect
      (and
        (at start (not (available ?r)))
        (at start (not (channel_free ?l)))
        (at end (channel_free ?l))
        (at end (communicated_soil_data ?p))
        (at end (available ?r))
        (at start (decrease (energy ?r) 4))
  	)
  )

  (:durative-action communicate_rock_data
    :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
    :duration (= ?duration 10)
    :condition
      (and
        (over all (at ?r ?x))
        (over all (at_lander ?l ?y))
        (at start (have_rock_analysis ?r ?p))
        (at start (can-see ?x ?y))
        (at start (available ?r))
        (at start (channel_free ?l))
        (at start (>= (energy ?r) 4))
      )
    :effect
      (and
        (at start (not (available ?r)))
        (at start (not (channel_free ?l)))
        (at end (channel_free ?l))
        (at end (communicated_rock_data ?p))
        (at end (available ?r))
        (at start (decrease (energy ?r) 4))
      )
  )

  (:durative-action communicate_image_data
    :parameters (?r - rover ?l - lander ?o - objective ?m - mode ?x - waypoint ?y - waypoint)
    :duration (= ?duration 15)
    :condition
      (and
        (over all (at ?r ?x))
        (over all (at_lander ?l ?y))
        (at start (have_image ?r ?o ?m))
        (at start (can-see ?x ?y))
        (at start (available ?r))
        (at start (channel_free ?l))
        (at start (>= (energy ?r) 6))
      )
    :effect
      (and
        (at start (not (available ?r)))
        (at start (not (channel_free ?l)))
        (at end (channel_free ?l))
        (at end (communicated_image_data ?o ?m))
        (at end (available ?r))
        (at start (decrease (energy ?r) 6))
      )
  )

  ;;Tareas compuestas
  (:task go-to
  	:parameters (?rov - rover ?wDest - waypoint)
    (:method still ;; Si el rover esta en el destino: Se esta quieto
      :precondition
        (and
          (at ?rov ?wDest)
        )
  	  :tasks ()
    )
    (:method move ;; Si el rover no esta en el destino y el objetivo esta adyacente: Se mueve
      :precondition
        (and
          (at ?rov ?wOr)
          (can_traverseBI ?rov ?wOr ?wDest)
          (can-see ?wOr ?wDest)
        )
      :tasks
        ((navigate ?rov ?wOr ?wDest))
    )
    (:method moveRecursive ;; Si el rover no esta en el destino y el objetivo esta adyacente: Se mueve
      :precondition
        (and
          (at ?rov ?wOr)
          (can_traverseBI ?rov ?wOr ?wInt)
          (can-see ?wOr ?wInt)
        )
      :tasks(
        (navigate ?rov ?wOr ?wInt)
        (go-to ?rov ?wDest)
      )
    )
  )


  (:task clear-storage
    :parameters (?rov - rover)
    (:method notFull ;; Si el rover esta en el vacio: Se esta quieto
      :precondition
        (and
          (store_of ?storage ?rov)
          (empty ?storage)
          (not (full ?storage))
        )
      :tasks()
    )
    (:method full ;; Si el rover esta en el lleno: Se vacía
      :precondition
        (and
          (store_of ?storage ?rov)
          (not (empty ?storage))
          (full ?storage)
        )
      :tasks
        ((drop ?rov ?storage))
      )
    )

  (:task refill-batt
    :parameters (?rov - rover)
    (:method move-and-charge ;; Si el no esta en una zona con disponibilidad de sol. Se mueve a una y recarga las baterias
      :precondition
        (and
          (at ?rov ?shadowywp)
          (not (in_sun ?shadowywp))
          (in_sun ?sunnywp)
          (different ?sunnywp ?shadowywp)
          (< (energy ?rov) 80)
        )
      :tasks (
        (go-to ?rov ?sunnywp)
        (recharge ?rov ?sunnywp)
      )
    )
    (:method charge ;; Si el rover esta en una zona soleada. Se pone al sol
      :precondition
        (and
          (at ?rov ?sunnywp)
          (in_sun ?sunnywp)
          (< (energy ?rov) 80)
        )
      :tasks
        ((recharge ?rov ?sunnywp))
    )
  )

  (:task calibrate-cam
    :parameters (?rov - rover ?obj - objective )
    (:method make-calibration ;; Si el no esta en una zona con disponibilidad de sol. Se mueve a una y recarga las baterias
      :precondition
        (and
          (calibration_target ?cam ?obj)
          (visible_from ?obj ?wp)
          (on_board ?cam ?rov)
          (not (calibrated ?cam ?rov))
        )
	    :tasks
        ((calibrate ?rov ?cam ?obj ?wp ))
    )
    (:method no-make-calibration ;; Si el no esta en una zona con disponibilidad de sol. Se mueve a una y recarga las baterias
      :precondition
        (and
          (calibration_target ?cam ?obj)
          (visible_from ?obj ?wp)
          (on_board ?cam ?rov)
          (calibrated ?cam ?rov)
        )
      :tasks()
    )
  )

  (:task communicate
    :parameters (?rov - rover)
    (:method commrocks ;;Comunicamos un sample de las rocas. Nos movemos y mandamos los datos al lander.
      :precondition
        (and
          (have_rock_analysis ?rov ?wpSampled)
          (at_lander ?landr ?wpLander)
          (can-see ?wp ?wpLander)
          (channel_free ?landr)
        )
      :tasks (
        (go-to ?rov ?wp)
        (communicate_rock_data ?rov ?landr ?wpSampled ?wp ?wpLander)
      )
    )
    (:method commsoil ;;Comunicamos un sample del suelo. Nos movemos y mandamos los datos al lander.
      :precondition
        (and
          (have_soil_analysis ?rov ?wpSampled)
          (at_lander ?landr ?wpLander)
          (can-see ?wp ?wpLander)
          (channel_free ?landr)
        )
      :tasks (
        (go-to ?rov ?wp)
        (communicate_soil_data ?rov ?landr ?wpSampled ?wp ?wpLander)
      )
    )
    (:method commphoto ;;Comunicamos una foto. Nos movemos y mandamos los datos al lander..
      :precondition
        (and
          (have_image ?rov ?obj ?pmode)
          (at_lander ?landr ?wpLander)
          (can-see ?wp ?wpLander)
          (channel_free ?landr)
        )
      :tasks (
        (go-to ?rov ?wp)
        (communicate_image_data ?rov ?landr ?obj ?pmode ?wp ?wpLander)
      )
    )
  )

  (:task sample
    :parameters (?rov - rover ?wp - waypoint)
    (:method rocks ;; Tomamos un sample de las rocas. Nos movemos, hacemos hueco en el inventario, la analizamos y la mandamos al lander.
      :precondition
        (and
          (at_rock_sample ?wp)
          (equipped_for_rock_analysis ?rov)
          (store_of ?storage ?rov)
        )
      :tasks (
        [(go-to ?rov ?wp)
        (clear-storage ?rov)]
        (sample_rock ?rov ?storage ?wp)
        (communicate ?rov)
      )
    )
  (:method soil ;; Tomamos un sample del suelo. Nos movemos, hacemos hueco en el inventario, la analizamos y la mandamos al lander.
    :precondition
      (and
        (at_soil_sample ?wp)
        (equipped_for_soil_analysis ?rov)
        (store_of ?storage ?rov)
      )
    :tasks (
      [(go-to ?rov ?wp)
      (clear-storage ?rov)]
      (sample_soil ?rov ?storage ?wp)
      (communicate ?rov)
    )
   )
  )

  (:task photo
    :parameters (?rov - rover ?obj - objective ?pmode - mode)
    (:method takePhoto ;;Tomamos una foto. Nos movemos, hacemos la foto y la mandamos.
      :precondition
        (and
          (equipped_for_imaging ?rov)
          (on_board ?cam ?rov)
          (supports ?cam ?pmode)
          (visible_from ?obj ?wDest)
        )
      :tasks (
        [(go-to ?rov ?wDest)
        (calibrate-cam ?rov ?obj)]
        (take_image ?rov ?wDest ?obj ?cam ?pmode)
        (communicate ?rov)
      )
    )
    (:method nottakePhoto ;;Si ya la tenemos, nos estamos quietos.
      :precondition
        (and
          (have_image ?rov ?obj ?pmode)
        )
      :tasks()
    )
  )
)
