; Domain from https://github.com/b-it-bots/mas_domestic_robotics/blob/devel/mdr_planning/mdr_rosplan_interface/config/default_domain.pddl

(define (domain service-robot-domain)

    (:requirements :strips :typing :equality)

    (:types
        Thing
        Object - Thing
        Location - Thing
        Waypoint - Location
        Furniture - Object
        Category
        Robot
        Door
        Plane
        Person
        NamedPose
        GraspingStrategy
        Context
    )

    (:constants pick_from_plane pick_from_container place_on_plane place_in_container - Context)

    (:predicates
        (robotName ?Robot - Robot)
        (objectCategory ?Object0 - Object ?Object1 - Object)
        (robotAt ?Robot - Robot ?Waypoint - Waypoint)
        (doorAt ?Door - Door ?Waypoint - Waypoint)
        (objectAt ?Object - Object ?Waypoint - Waypoint)
        (furnitureAt ?Furniture - Furniture ?Waypoint - Waypoint)
        (planeAt ?Plane - Plane ?Waypoint - Waypoint)
        (personAt ?Person - Person ?Waypoint - Waypoint)
        (doorOpen ?Door - Door)
        (belongsTo ?Plane - Plane ?Object - Object)
        (unexplored ?Plane - Plane)
        (explored ?Plane - Plane)
        (objectOnPlane ?Object - Object ?Plane - Plane)
        (objectOnObject ?Object0 - Object ?Object1 - Object)
        (objectInObject ?Object0 - Object ?Object1 - Object)
        (objectInFurniture ?Object - Object ?Furniture - Furniture)
        (robotHoldingObject ?Robot - Robot ?Object - Object)
        (personHoldingObject ?Person - Person ?Object - Object)
        (emptyGripper ?Robot - Robot)
        (known ?Person - Person)
        (unknown ?Person - Person)

        (canPlaceOn ?Object - Object ?Plane - Plane)
        (defaultStoringLocation ?Object - Object ?Furniture - Furniture)
        (likelyLocation ?Object - Object ?Furniture - Furniture)
        (locatedAt ?Object - Object ?Location - Location)
        (hasDoor ?Furniture - Furniture)
        (above ?Object0 - Object ?Object1 - Object)
        (below ?Object0 - Object ?Object1 - Object)
        (onTopOf ?Object0 - Object ?Object1 - Object)
        (inside ?Object0 - Object ?Object1 - Object)
        (toTheLeftOf ?Object0 - Object ?Object1 - Object)
        (toTheRightOf ?Object0 - Object ?Object1 - Object)
        (isAtNamedPose ?Thing - Thing ?NamedPose - NamedPose)
        (isAtLocation ?NamedPose - NamedPose ?Location - Location)
        (preferredGraspingStrategy ?Object - Object ?GraspingStrategy - GraspingStrategy)
    )

    (:action MoveBase
        :parameters (?Robot - Robot ?Location0 ?Location1 - Location)
        :precondition (and
            (robotAt ?Robot ?Location0)
        )
        :effect (and
            (not (robotAt ?Robot ?Location0))
            (robotAt ?Robot ?Location1)
        )
    )

    (:action Open
        :parameters (?Door - Door ?Robot - Robot ?Waypoint - Waypoint)
        :precondition (and
            (doorAt ?Door ?Waypoint)
            (robotAt ?Robot ?Waypoint)
        )
        :effect (and
            (doorOpen ?Door)
        )
    )

    (:action PerceivePlane
        :parameters (?Plane - Plane ?Robot - Robot ?Waypoint - Waypoint)
        :precondition (and
            (robotAt ?Robot ?Waypoint)
            (planeAt ?Plane ?Waypoint)
            (unexplored ?Plane)
        )
        :effect (and
            (not (unexplored ?Plane))
            (explored ?Plane)
        )
    )

    (:action PickFromPlane
        :parameters (?Object - Object ?Plane - Plane ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context pick_from_plane)
            (robotAt ?Robot ?Waypoint)
            (planeAt ?Plane ?Waypoint)
            (explored ?Plane)
            (objectOnPlane ?Object ?Plane)
            (emptyGripper ?Robot)
        )
        :effect (and
            (not (objectOnPlane ?Object ?Plane))
            (not (emptyGripper ?Robot))
            (robotHoldingObject ?Robot ?Object)
        )
    )

    (:action PickFromContainer
        :parameters (?Object - Object ?Furniture - Furniture ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context pick_from_container)
            (robotAt ?Robot ?Waypoint)
            (furnitureAt ?Furniture ?Waypoint)
            (objectInFurniture ?Object ?Furniture)
            (emptyGripper ?Robot)
        )
        :effect (and
            (not (objectInFurniture ?Object ?Furniture))
            (not (emptyGripper ?Robot))
            (robotHoldingObject ?Robot ?Object)
        )
    )

    (:action PlaceOnPlane
        :parameters (?Object - Object ?Plane - Plane ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context place_on_plane)
            (robotAt ?Robot ?Waypoint)
            (planeAt ?Plane ?Waypoint)
            (robotHoldingObject ?Robot ?Object)
        )
        :effect (and
            (not (robotHoldingObject ?Robot ?Object))
            (emptyGripper ?Robot)
            (objectOnPlane ?Object ?Plane)
        )
    )

    (:action PlaceInContainer
        :parameters (?Object - Object ?Furniture - Furniture ?Robot - Robot ?Waypoint - Waypoint ?Context - Context)
        :precondition (and
            (= ?Context place_in_container)
            (robotAt ?Robot ?Waypoint)
            (furnitureAt ?Furniture ?Waypoint)
            (robotHoldingObject ?Robot ?Object)
        )
        :effect (and
            (not (robotHoldingObject ?Robot ?Object))
            (emptyGripper ?Robot)
            (objectInFurniture ?Object ?Furniture)
        )
    )

    (:action Throw
        :parameters (?Object0 ?Object1 - Object ?Robot - Robot ?Waypoint - Waypoint)
        :precondition (and
            (robotAt ?Robot ?Waypoint)
            (objectAt ?Object1 ?Waypoint)
            (robotHoldingObject ?Robot ?Object0)
        )
        :effect (and
            (not (robotHoldingObject ?Robot ?Object0))
            (emptyGripper ?Robot)
            (objectInObject ?Object0 ?Object1)
        )
    )

    (:action HandOver
        :parameters (?Object - Object ?Robot - Robot ?Person - Person ?Waypoint - Waypoint)
        :precondition (and
            (robotAt ?Robot ?Waypoint)
            (personAt ?Person ?Waypoint)
            (robotHoldingObject ?Robot ?Object)
        )
        :effect (and
            (not (robotHoldingObject ?Robot ?Object))
            (personHoldingObject ?Person ?Object)
        )
    )
)
