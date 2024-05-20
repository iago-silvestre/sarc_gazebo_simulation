{ include("mission-management2.asl", mm) }
traj([[-5,-5,5],[5,-5,5],[5,5,5],[-5,5,5]]).
mynumber(1).
current_mission("None").
myautopilot(autopilot).

!my_missions.
+!my_missions
   <- !mm::create_mission(pa, 900, []); // scan
      +mm::mission_plan(pa,[[-5,-5,5],[5,-5,5],[5,5,5],[-5,5,5]]); // a list of waypoints
      !mm::create_mission(pb, 100, [drop_when_interrupted,loop]); // extinguish
      +mm::mission_plan(pb,[[5,-8,5],[0,-8,5]]);
      // go home
      // land now


      !mm::run_mission(pa);
      .wait(33000);
      !mm::run_mission(pb).



+fire <- !mm::run_mission(pb).
-energy <- !mm::run_mission(gohome).

+mm::mission_state(Id,S) // "callback" when a mission is finished
   <- .print("Mission ",Id," state is ",S).

+mm::mission_loop(Id) 
   <- .send(autopilot,tell,mission_loop(Id)).

+mm::current_mission(Id)
   <- //.print("Current Mission :",Id);
      //-current_mission(_);
      //+current_mission(Id);
      .send(autopilot,tell,update_current_mission(Id)).//    <- No need for autopilot to ask for it, test this


/*+whats_my_current_mission[source(A)]                   //Unnecessary,mm::current_mission takes care of it
   : current_mission(Id)
  <- //.print("Current Mission :",Id);
     .send(A,tell,update_current_mission(Id));
     -whats_my_current_mission[source(A)].*/