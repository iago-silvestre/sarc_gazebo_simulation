+!found_fire
   : current_position(CX, CY, CZ) & std_altitude(Z) & my_number(N)
   & fireSize(FS) & frl_charges(FRL) & FS > FRL
   & current_mission(search)
   <- +fire_pos(CX,CY);
      .print("Fire detected in X: ",CX," , Y:",CY);
      .print("FRL dif: ",(FS-FRL));
      !mm::create_mission(combat_fire, 10, [drop_when_interrupted]);
      ?combat_traj(CT);
      +mm::mission_plan(combat_fire,CT);
      !mm::run_mission(combat_fire);
	  .broadcast(tell,help_fire_pos(CX,CY)).

+mm::mission_state(combat_fire,finished)   
   : fireSize(FS) & FS==0
   <- .print("Fire Extinguished").

+mm::mission_state(combat_fire,finished) 
   : frl_charges(FRL) & FRL>=1
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","fightFire",FRL);
      -+frl_charges(FRL-1);
      .wait(1000);
      !mm::run_mission(combat_fire).
	  
+help_fire_pos(CX,CY)[source(A)]
   :  std_altitude(Z)
   <- !mm::create_mission(goto_fire, 10, []); 
      +mm::mission_plan(goto_fire,[[CX,CY,Z+N]]);
      !mm::run_mission(goto_fire).

+mm::mission_state(goto_fire,finished)  
   : fire_pos(CX,CY) & std_altitude(Z) & my_number(N) & combat_traj(CT)
   <- .print("Go to fire finished!");
      !mm::create_mission(combat_fire, 10, [drop_when_interrupted]);
      +mm::mission_plan(combat_fire,CT);
      !mm::run_mission(combat_fire).