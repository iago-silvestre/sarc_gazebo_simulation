+!stop_mission
   <- .print("**** stop_mission is not implemented, just select another mission.").

+!run_plan(CM,L)[source(Ag)] 
   : my_number(N)
   <- -+my_agent(Ag);
      -+current_mission(CM);
      -+mission_plan(CM,L);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","path",[N,L] ).

+uav_lastWP(N) 
   : current_mission(CM) & my_agent(Ag)
   <- -progress(CM,_);
      +progress(CM,N);
      .send(Ag,signal,update_rem_plan(N,0)).

+progress(CM,N) 
   : not mission_loop(CM) & mission_plan(CM,Plan) & .length(Plan,N) & my_agent(Ag)
   <- -progress(CM,_);
      +progress(CM,0);
      .send(Ag,signal,finished).
