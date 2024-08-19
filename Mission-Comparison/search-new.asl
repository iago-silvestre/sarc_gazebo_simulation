+!my_missions
   :  waypoints_list(L) & my_number(N) 
   <- !mm::create_mission(search, 10, []); 
      +mm::mission_plan(search,L); 
      !mm::run_mission(search).