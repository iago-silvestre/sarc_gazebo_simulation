+!follow_trajectory(CW)
   :  waypoints_list_len(CW)
      & my_number(N)
   <- .broadcast(tell, finished_trajectory(N));
      +finished_trajectory(N);
      -+status("finished_trajectory");
      .print("finished_trajectory");
      !wait_for_others.

+!follow_trajectory(CW)
   :  waypoints_list(WL)
      & waypoints_list_len(Len)
      & CW < Len
   <- -+status("following_trajectory");
      .print("following_trajectory");
      .nth(CW, WL, [X, Y, Z]);
      !check_near(X, Y, Z, "waypoint");
      !follow_trajectory(CW+1).