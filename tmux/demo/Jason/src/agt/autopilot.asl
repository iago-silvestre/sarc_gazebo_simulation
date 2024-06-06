/*last_mission("None").
current_mission("Idle").
progress(pa,0).
progress(pb,0).
progress(pc,0).
*/

//+uav_lastWP(N) : current_mission(CM) <- -+progress(CM,N).

+!stop_mission
   <- .print("**** stop_mission is not implemented, just select another mission.");
   .

/*+!run_plan(L)[source(Ag)]
   : current_mission(CM)
     & progress(CM,LX) & (LX == .length(L))
     //& progress(CM,LX) & .delete(-1,LX,L,SL) & (LX == .length(SL))
   <- .print("Finished Mission :", CM);
      .send(Ag,signal,finished).
*/

+!run_plan(CM,L)[source(Ag)] 
   : my_number(N)
     //& current_mission(CM) //& last_mission(LM) & not(CM == LM)
     //& progress(CM,LX) & not (LX == .length(L))
   <- //.delete(-1,LX,L,SL);
      -+my_agent(Ag);
      -+current_mission(CM);
      -+mission_plan(CM,L);
      if (mission_loop(CM)) {
         embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_light_loop",[N,L] );
      } else {
         embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_light",[N,L] );
      }
      //.print("SubList :",SL);
      //.print("SubList Size :",.length(SL));
      //-+last_mission(CM);
      //!run_plan(SL)[source(Ag)].
   .

/*
+!run_plan(L)[source(Ag)] 
   : current_mission(CM) & last_mission(LM) & (CM == LM)
   & progress(CM,LX) & not (LX == .length(L))
   <- .print("Current Mission :",CM," Current Progress :",LX," List Length :",.length(L));
      .wait(1000);
      !run_plan(L)[source(Ag)].
*/

/*+update_current_mission(Id)[source(Ag)] 
  <- -current_mission(_);
     +current_mission(Id);
     -update_current_mission(Id)[source(Ag)].
*/


+uav_lastWP(N) 
   : current_mission(CM) 
   <- -progress(CM,_);
      //.print("Current Mission :",CM," Current Progress :",N);
      +progress(CM,N).

+progress(CM,N) 
   : not mission_loop(CM) & mission_plan(CM,Plan) & .length(Plan,N) & my_agent(Ag)
   <- -progress(CM,_);
      +progress(CM,0);
      .send(Ag,signal,finished).

/*+progress(CM,N) 
   : my_agent(Ag)
   <- UsedEnergy = 0;
      .send(Ag,achieve,update_rem_plan(N,UsedEnergy)).*/

// *** Energy

/*+!do(Step,Ag,Rem)
   <- .print("doing ",Step);
      UsedEnergy = 5;
      .send(Ag,achieve,update_rem_plan(Step,UsedEnergy));
      .wait(1000);
   .   
*/

/*!consume_energy(1000). // it initially has 1000 of energy

+!consume_energy(E)    // that decais 10 units each second
   <- .send(sample_agent,achieve,update_energy(E));
      .wait(1000);
      !consume_energy(E-10).
*/
//+!run_plan(_)[source(Ag)] .