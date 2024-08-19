+mm::mission_state(search,finished) 
   : my_number(N) & current_position(CX, CY, CZ) 
   <- .broadcast(tell, finished_trajectory(N));
      !wait_for_others.

+mm::mission_state(waiting,finished) 
   <- !wait_for_others.

+!wait_for_others
   :  my_landing_position(LAX, LAY) & std_altitude(Z)
      & .count(finished_trajectory(_), C) & nb_participants(C)
   <- .print("All finished, going to land position");
      !mm::create_mission(goto_land, 10, []); 
      +mm::mission_plan(goto_land,[[LAX,LAY,Z]]);
      !mm::run_mission(goto_land).

+!wait_for_others 
   <-.wait(1000);
      !wait_for_others.

+mm::mission_state(goto_land,finished) 
   <- .print(" Arrived at landing point, landing!");
       embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","land",[]).