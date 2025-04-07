+!create_mission(Id,ExpEnergy, Args) 
   <- +mission_energy(Id,ExpEnergy,0);
      if ( .member(drop_when_interrupted, Args)) {
        +mission_drop_when_interrupted(Id);
      }
      if ( .member(loop, Args)) {
        +mission_loop(Id);
      }      
   .

+mission_loop(Id)
   : my_ap(AP)
   <- .send(AP,tell,mission_loop(Id)).

@[atomic] +!run_mission(Id) : current_mission(Id). 
@[atomic] +!run_mission(Id)  
   :  not current_mission(_) & 
      mission_state(Id,suspended) &
      mission_step(Id,Step) &
      mission_plan(Id,Plan)  &
      my_ap(AP)
   <- .delete(0,Step,Plan,RemPlan); .print("Resuming ",Plan,", remaining plan is ",RemPlan," (",Step,"/",.length(Plan),")");
      +current_mission(Id);
      !change_state(Id,running);
      .send(AP,achieve,run_plan(Id,RemPlan)).
@[atomic] +!run_mission(Id) 
   :  not current_mission(_) & 
      mission_plan(Id,Plan) &
      my_ap(AP)
   <- +current_mission(Id);
      !change_state(Id,running);
      .send(AP,achieve,run_plan(Id,Plan)).
@[atomic] +!run_mission(Id) 
   :  current_mission(CMission) & 
      CMission \== Id  &
      my_ap(AP)
   <- .send(AP,achieve,stop_mission);
      -current_mission(CMission);
      if (mission_drop_when_interrupted(CMission)) {
         !change_state(CMission,dropped);
      } else {
         !change_state(CMission,suspended);
      }
      !run_mission(Id).

@[atomic] +!stop_mission(Id,R)
   :  current_mission(Id)  &
      my_ap(AP)
   <- .send(AP,achieve,stop_mission);
      -current_mission(Id);
      !change_state(Id,stopped[reason(R)]);
      !auto_resume.

@[atomic] +!drop_mission(Id,R)
   :  current_mission(Id)  &
      my_ap(AP)
   <- .send(AP,achieve,stop_mission);
      -current_mission(Id);
      !change_state(Id,dropped[reason(R)]);
      !auto_resume.

@[atomic] +!stop_mission(Id,R)
   <- !auto_resume.

+!start_mission(Id)
   :  enough_energy(Id) &
      mission_plan(Id,Plan)  &
      my_ap(AP)
   <- .send(AP,achieve,run_plan(Id,Plan)).

+default::finished
   <- -current_mission(Id);
      !change_state(Id,finished);
      !auto_resume.   

+default::update_rem_plan(Step,Energy)
   :  current_mission(Mission) & 
      mission_energy(Mission,EE,US) 
   <- -mission_step(Mission,_);
      +mission_step(Mission,Step);
      -mission_energy(Mission,EE,US);
      +mission_energy(Mission,EE,US+Energy).

+!auto_resume
   :  not current_mission(_) &
      mission_state(Mission,suspended) &
      enough_energy(Mission)
   <- !run_mission(Mission).
+!auto_resume.

+!change_state(Mission, State) 
   <- -mission_state(Mission, _);
      +mission_state(Mission, State).


+!default::update_energy(E)
   <- -+available_energy(E).
      
available_energy(100000000). 

enough_energy(RE)      :- .number(RE) & available_energy(A) & A > RE.
enough_energy(Mission) :- mission_energy(Mission,EE,US) & available_energy(A) & A > (EE-US).

+available_energy(_)    <- !test_enough_energy.
+mission_energy(Mission,EE,US) <- !test_enough_energy.

+!test_enough_energy
    : current_mission(Mission) & 
      mission_energy(Mission,EE,US) & 
      Rem = math.max(0,EE-US) &
      not enough_energy(Rem)
   <- ?available_energy(A);
      !!stop_mission(Mission,lack_of_energy(required(Rem),available(A))). 
+!test_enough_energy.
