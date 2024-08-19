+!follow_trajectory(CW)
   :  waypoints_list_len(CW)
      & my_number(N)
   <- .broadcast(tell, finished_trajectory(N));
      +finished_trajectory(N);
      -+status("finished_trajectory");
      .print("finished_trajectory");
      !wait_for_others.

+!wait_for_others
   :  my_number(N)
      & my_landing_position(LAX, LAY)
      & .count(finished_trajectory(_), C)
      & num_of_uavs(C)
   <- .print("All finished, going to land position");
      !goto_landing_position(LAX, LAY).

+!wait_for_others
   <- -+status("waiting");
      .print("Still waiting");
      .wait(1000);
      !wait_for_others.

+!goto_landing_position(X, Y)
   : std_altitude(Z)
   <- -+status("going_to_land_position");
      !check_near(X, Y, Z, "land position");
      !land.

+!land
   :  my_number_string(N)
   <- .print("Landing");
      -+status("landing");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1", "land", [N]).