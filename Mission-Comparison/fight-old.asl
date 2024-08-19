+!detected_fire(N)
   :  my_number(N)
      & current_position(CX, CY, CZ)
      & .intend(follow_trajectory(CW))
      & not status("combating_fire")
      & not fire_extinguished
   <- .suspend(follow_trajectory(CW));
      -+status("combating_fire");
      .print("Fire found by ", N, ". Suspending trajectory.");
      .broadcast(tell, found_fire(N, CX, CY));
      !combat_fireR(CW).

+found_fire(N, X, Y)
   : not my_number(N)
      & .intend(follow_trajectory(CW))
      & not status("combating_fire")
      & not fire_extinguished
   <- .suspend(follow_trajectory(CW));
      -+status("combating_fire");
      .print("Fire found by ", N, ". Suspending trajectory.");
      !goto_fire_position(X+N, Y, 15);
      !combat_fireR(CW).

+found_fire(N, X, Y)
   : not my_number(N)
      & .intend(wait_for_others)
      & not status("combating_fire")
      & not fire_extinguished
   <- .suspend(wait_for_others);
      -+status("combating_fire");
      .print("Fire found by ", N, ". Suspending waiting.");
      !goto_fire_position(X+N, Y, 15);
      !combat_fire.
      

+!goto_fire_position(X, Y, Z)
   <- !check_near(X, Y, Z, "fire position").
   
+!combat_fireR(CW)
   : frl_charges(0)
   <- .wait(10000);
      .print("No more Fire Retardant, going to recharge");
      .suspend(combat_fireR(CW));
      !recharge_frl.
	  
      
+!combat_fireR(CW)
   : fireSize(1)
   <- .wait(10000);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[CX, CY, 8.0]);
      +fire_extinguished;
      .resume(follow_trajectory(CW));
      .print("Fire extinguished. Resuming trajectory").   
	  
+!combat_fireR(CW)
   : fireSize(FS) & FS>1 & frl_charges(FRL)
   <- .wait(10000);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[CX, CY, 8.0]);
      -+frl_charges(FRL-1);
      !combat_fireR.