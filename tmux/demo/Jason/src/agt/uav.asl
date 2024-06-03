{ include("mission-management2.asl", mm) }

current_mission("None").
status("None").
world_area(100, 100, 0, 0).
num_of_uavs(4).
camera_range(5).
std_altitude(7.0).
std_heading(0.0).
land_radius(10.0).
frl_charges(1).
//fire_size(9).
//fireSize(4).

nb_participants(3).

//currentwaypoint(0).

temp_limit(70.5).//30.0
wind_limit(72.5).//12.1
//fireLoc(24.5, -23.5).
landing_x(0.0).
landing_y(0.0).

current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(1) & uav1_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(2) & uav2_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(3) & uav3_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(4) & uav4_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).

my_ap(AP) :- my_number(N)
            & .term2string(N, S) & .concat("autopilot",S,AP).

distance(X,Y,D) :- current_position(CX, CY, CZ) & D=math.sqrt( (CX-X)**2 + (CY-Y)**2 ).

+fire_detection(N) : N>=6000 <- !found_fire.
//+fireSize(FS) <- -fireSize(_); +fireSize(FS). //infinite loop 
//////////////// Start
!start.

+fireSize(FS)
   : FS == 0 & current_mission(combat_fire)
   <- !mm::drop_mission(combat_fire,"Fire is Extinguished").
      
+fireSize(FS)
   : FS == 0 & current_mission(goto_fire)
   <- !mm::drop_mission(goto_fire,"Fire is Extinguished").

+!start
   :my_ap(AP)
    <- .wait(200);
      +mm::my_ap(AP);
       //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1", "land",[]);
      //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[0.0, 0.0, 0.0]);
      .print("Started!");
      !calculate_trajectory;//trajectory//!calculate_area;//!calculate_waypoints(1, []);// pode ser unido com os outros
      //!hover.
      !my_missions.



+!my_missions
   :  waypoints_list(L) & my_number(N) & N==1
   <- !mm::create_mission(search, 900, []); // scan
      //+mm::mission_plan(search,L); // a list of waypoints

      //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_light",[N,L] );
      +mm::mission_plan(search,[[23,-23,7],[50,-50,7]]); // a list of waypoints
     // !mm::create_mission(pb, 100, [drop_when_interrupted]); // extinguish
     // !mm::create_mission(pb, 100, [drop_when_interrupted,loop]); // extinguish
      //+mm::mission_plan(pb,[[5,-8,5],[0,-8,5]]);
      // go home
      // land now


      !mm::run_mission(search).
      //.wait(8000);
      //!mm::drop_mission(combat_fire,"debug").
      //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","stop_tracking",[]);  
      //+found_fire(5,5).
      //!low_battery.

+!my_missions
   :  waypoints_list(L) & my_number(N) & not (N==1)
   <- !mm::create_mission(search, 900, []); // scan
      +mm::mission_plan(search,L); // a list of waypoints
      !mm::run_mission(search).

+fire <- !mm::run_mission(pb).
-energy <- !mm::run_mission(gohome).


+frl_charges(N)
   : N==0
   <- .print(" No more Fire Retardant charges, going to recharge");
      //!mm::stop_mission(combat_fire,"Recharging");
      !mm::create_mission(low_frl, 900, []); // Recharge Battery
      +mm::mission_plan(low_frl,[[0,0,10]]);
      !mm::run_mission(low_frl).

+!low_battery
   <- .print(" Low Battery, going back to Recharge");
      !mm::create_mission(low_batt, 900, []); // Recharge Battery
      +mm::mission_plan(low_batt,[[0,0,10]]);
      !mm::run_mission(low_batt).

+mm::mission_state(low_batt,finished) 
   <- .print(" Recharging Battery");
      .wait(10000);
      .print(" Recharged!!").  //Still need to publish rechargeBattery topic


      

+mm::mission_state(low_frl,finished) 
   <- .print(" Recharging FRL");
      .wait(10000);
      .print(" Recharged!!");  //Still need to publish rechargeBattery topic
      -+frl_charges(4);
      !analyzeFire.

+!analyzeFire
   : fireSize(FS) & FS >0 & fire_pos(CX,CY)
   <- .print(" Going back to fire!!");
      !mm::create_mission(goto_fire, 900, []); // gotofire
      +mm::mission_plan(goto_fire,[[CX,CY,7]]);
      !mm::run_mission(goto_fire).

+!found_fire
   : current_position(CX, CY, CZ) & std_altitude(Z)
   & fireSize(FS) & frl_charges(FRL) & FS > FRL
   & current_mission(search)
   <- .print("Need help for detected fire in : ",CX," , ",CY);
      .print("FRL Needed: ",(FS-FRL));
      +fire_pos(CX,CY);
      !mm::create_mission(combat_fire, 100, [drop_when_interrupted]);
      +mm::mission_plan(combat_fire,[[CX-2,CY+2,Z],[CX+2,CY+2,Z],[CX+2,CY-2,Z],[CX-2,CY-2,Z]]);
      //+mm::mission_plan(combat_fire,[[CX,CY+1.5,Z],[CX+1.5,CY,Z],[CX,CY-1.5,Z],[CX-1.5,CY,Z]]);
      !mm::run_mission(combat_fire);
      !cnp( 2,help,(FS-FRL)).

+mm::mission_state(combat_fire,finished)   // Priority
   : fireSize(FS) & FS==0
   <- .print("Fire Extinguished").


+mm::mission_state(combat_fire,finished) 
   : frl_charges(FRL) & FRL>1
   <- .print("Loop finished!,Remaining Charges", (FRL-1));
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","fightFire",FRL);
      -+frl_charges(FRL-1);
      .wait(200);
      //!mm::create_mission(combat_fire, 100, [drop_when_interrupted]);
      //+mm::mission_plan(combat_fire,[[CX-5,CY+5,5],[CX+5,CY+5,5],[CX+5,CY-5,5],[CX-5,CY-5,5]]);
      !mm::run_mission(combat_fire).

+mm::mission_state(combat_fire,finished) 
   : frl_charges(FRL) & FRL==1
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","fightFire",FRL);
      -+frl_charges(FRL-1).


price(_Service,X,Y,R) :- 
   current_position(X, Y, CZ) & 
   frl_charges(R). //math.floor(R*20)+1.

// answer to Call For Proposal
@c1 +cfp(CNPId,Task)[source(A)]
   :  price(Task,X,Y,R)
   <- +proposal(CNPId,Task,X,Y,R); // remember my proposal
      .send(A,tell,propose(CNPId,X,Y,R)).

@r1 +accept_proposal(CNPId)[source(A)]
   :  proposal(CNPId,Task,X,Y,R) & fire_pos(CX,CY) & std_altitude(Z)
   <- .print("My proposal '",R,"' was accepted for CNP ",CNPId, ", task ",Task," for agent ",A,"!");
      .print("Going to fire in : ",CX," , ",CY);
      
      /*.print("Debugging CNP");
      !mm::create_mission(pa, 900, []); // scan
      +mm::mission_plan(pa,[[-5,-5,5],[5,-5,5],[5,5,5],[-5,5,5]]); // a list of waypoints
      !mm::run_mission(pa).*/

      //!mm::create_mission(combat_fire, 100, [drop_when_interrupted]);
      //+mm::mission_plan(combat_fire,[[CX-2,CY+2,Z],[CX+2,CY+2,Z],[CX+2,CY-2,Z],[CX-2,CY-2,Z]]);
      //!mm::run_mission(combat_fire).
      
      
      !mm::create_mission(goto_fire, 900, []); // gotofire
      +mm::mission_plan(goto_fire,[[CX,CY,7]]);
      !mm::run_mission(goto_fire).
   
@r2 +reject_proposal(CNPId)
   <- .print("My proposal was not accepted for CNP ",CNPId, ".");
      -proposal(CNPId,_,_,_,_). // clear memory






+!cnp(Id,Task,TR)
   <- !call(Id,Task);
      !bids(Id,LO,TR);
      !result(Id,LO,TR).
+!call(Id,Task)
   : current_position(CX, CY, CZ) 
   <- .broadcast(tell,cfp(Id,Task));
      .broadcast(tell,fire_pos(CX,CY)).
+!bids(Id,LOS,TR) // the deadline of the CNP is now + 3 seconds (or all proposals received)
    : nb_participants(LP)
   <- .wait(all_proposals_received(Id,LP), 3000, _);
      .findall( offer(U,R,D,A),
                propose(Id,X,Y,R)[source(A)] & distance(X,Y,D) & U=math.abs(TR-R),
                LO);
      .sort(LO,LOS);
      .print("Offers are ",LOS).

+!result(_,[],_).
+!result(CNPId,[offer(_,R,_,WAg)|T],RT) // announce result to the winner
    : RT > 0
   <- .send(WAg,tell,accept_proposal(CNPId));
      ND = RT-R;
      .findall(
          offer(NU,R1,D1,A1),
          .member(offer(N1,R1,D1,A1),T) & NU=math.abs(ND-R1),
          LO);
      .sort(LO,LOS);
      //.print("New list of offers ",LOS);
      !result(CNPId,LOS,ND).
+!result(CNPId,[offer(_,_,_,LAg)|T],RT) // announce to others
   <- .send(LAg,tell,reject_proposal(CNPId));
      !result(CNPId,T,RT).

/* Initial beliefs and rules */

all_proposals_received(CNPId,NP) :-              // NP = number of participants
     .count(propose(CNPId,_,_,_)[source(_)], NO) &   // number of proposes received
     .count(refuse(CNPId)[source(_)], NR) &      // number of refusals received
     NP = NO + NR.


/*+found_fire(CX,CY)
   : my_number(N)
   <- .print("Found fire in : ",CX," , ",CY);
      !mm::create_mission(goto_fire, 900, []); // gotofire
      +mm::mission_plan(goto_fire,[[CX,CY,10]]);
      !mm::run_mission(goto_fire).*/

+mm::mission_state(goto_fire,finished)  //goto fire finished
   : fire_pos(CX,CY) & std_altitude(Z)
   <- .print("Go to fire finished!");
      !mm::create_mission(combat_fire, 100, [drop_when_interrupted]);
      +mm::mission_plan(combat_fire,[[CX-2,CY+2,Z],[CX+2,CY+2,Z],[CX+2,CY-2,Z],[CX-2,CY-2,Z]]);
      !mm::run_mission(combat_fire).





+!calculate_trajectory
   :  my_number(N)
      & landing_x (LX)
      & landing_y (LY)
      & land_radius(R)
      & num_of_uavs(NumOfUavs)
      & world_area(H, W, CX, CY)
   <- .print("Calculating landing position");
      -+status("calculating_land_position");
      LndNumOfColumns = NumOfUavs/2;
      LndRectangleHeight = R/2;
      LndRectangleWidth = R/LndNumOfColumns;
      My_landing_x = LX - R/2 + LndRectangleWidth/2 + ((N-1) mod LndNumOfColumns)*LndRectangleWidth;
      My_landing_y = LY - R/2 + LndRectangleHeight/2 + (math.floor((N-1)/LndNumOfColumns))*LndRectangleHeight;
      +my_landing_position(My_landing_x, My_landing_y);
      //////////////// Calculating area
      .print("Calculating area");
      +status("calculating_area");
      AreaNumOfColumns = NumOfUavs/2;
      AreaRectangleHeight = H/2;
      AreaRectangleWidth = W/AreaNumOfColumns;
      X1 = CX - W/2 + ((N-1) mod AreaNumOfColumns)*AreaRectangleWidth;
      X2 = CX - W/2 + ((N-1) mod AreaNumOfColumns + 1)*AreaRectangleWidth;
      Y1 = CY - H/2 + (math.floor((N-1)/AreaNumOfColumns))*AreaRectangleHeight;
      Y2 = CY - H/2 + (math.floor((N-1)/AreaNumOfColumns) + 1)*AreaRectangleHeight;
      +my_area(X1, X2, Y1, Y2);
      //////////////// Calculating waypoints
      !calculate_waypoints(1, []).



//////////////// Calculating waypoints
+!calculate_waypoints(C, OldWayList)
    :   camera_range(CR)
        & my_area(X1, X2, Y1, Y2)
        & X2 - (C+2)*CR/2 >= X1
        & std_altitude(Z)
    <-  .print("Calculating waypoints");
        -+status("calculating_waypoints");
        Waypoints = [
                        [X1 + C*CR/2, Y1 + CR/2, Z]
                        , [X1 + C*CR/2, Y2 - CR/2, Z]
                        , [X1 + (C+2)*CR/2, Y2 - CR/2, Z]
                        , [X1 + (C+2)*CR/2, Y1 + CR/2, Z]
                    ];
        .concat(OldWayList, Waypoints, NewWayList);
        !calculate_waypoints(C+4, NewWayList).

+!calculate_waypoints(_, WayList)
    <-  .print("Finished calculating waypoints");
        +waypoints_list(WayList);
        +waypoints_list_len(.length(WayList));
        .print("Waypoints list: ", WayList).

+mm::mission_state(Id,S) // "callback" when a mission is finished
   <- .print("Mission ",Id," state is ",S).

+mm::current_mission(Id)
   <- //.print("Current Mission :",Id);
      -current_mission(_);
      +current_mission(Id).

/*+whats_my_current_mission[source(A)]                   //Unnecessary,mm::current_mission takes care of it
   : current_mission(Id)
  <- //.print("Current Mission :",Id);
     .send(A,tell,update_current_mission(Id));
     -whats_my_current_mission[source(A)].*/



+!found_fire.
+!my_missions.
+!analyzeFire.