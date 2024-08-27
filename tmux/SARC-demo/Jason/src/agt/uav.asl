{ include("mission-management2.asl", mm) }

current_mission("None").
status("None").
world_area(100, 100, 0, 0).
num_of_uavs(12).
nb_participants(11).
camera_range(5).
std_altitude(6.25).
std_heading(0.0).
land_radius(10.0).
frl_charges(1).
cnp_limit(0).
landing_x(0.0).
landing_y(0.0).
wind_speed(-20.0).
fire_pos(0.0,0.0).

current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(1) & uav1_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(2) & uav2_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(3) & uav3_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(4) & uav4_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(5) & uav5_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(6) & uav6_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(7) & uav7_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(8) & uav8_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(9) & uav9_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(10) & uav10_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(11) & uav11_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(12) & uav12_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).

combat_traj(CT) :- wind_speed(WS) & WS >=0.0 & fire_pos(CX,CY) & std_altitude(Z)  & my_number(N)
                  & CT= [[CX-2,CY+2,Z+N],[CX+2,CY+2,Z+N],[CX+2,CY-2,Z+N],[CX-2,CY-2,Z+N]].
combat_traj(CT) :- wind_speed(WS) & WS < 0.0 & fire_pos(CX,CY) & std_altitude(Z)  & my_number(N) 
                  & CT= [[CX-2,CY-2,Z+N],[CX+2,CY-2,Z+N],[CX+2,CY+2,Z+N],[CX-2,CY+2,Z+N]].


my_ap(AP) :- my_number(N)
            & .term2string(N, S) & .concat("autopilot",S,AP).

distance(X,Y,D) :- current_position(CX, CY, CZ) & D=math.sqrt( (CX-X)**2 + (CY-Y)**2 ).

+fire_detection(N) : N>=22000 <- !found_fire.
+battery(B) : B<=30.0 & not(low_batt) <- !low_battery.
//////////////// Start
!start.

+fireSize(0)
   : current_mission(combat_fire)
   <- !mm::stop_mission(combat_fire,"Fire is Extinguished").
      
+fireSize(0)
   : current_mission(goto_fire)
   <- !mm::stop_mission(goto_fire,"Fire is Extinguished").

+!start
   : my_ap(AP) & my_number(N) & combat_traj(CT)
    <- .wait(2000);
      +mm::my_ap(AP);
      .print("Started!",CT);
      !calculate_trajectory;//trajectory//!calculate_area;//!calculate_waypoints(1, []);// pode ser unido com os outros
      !my_missions.

+!my_missions
   :  waypoints_list(L) & my_number(N) //& N==1
   <- !mm::create_mission(search, 10, []); // scan
      +mm::mission_plan(search,L); // a list of waypoints
      !mm::run_mission(search).

+!my_missions
   :  waypoints_list(L) & my_number(N) & not (N==1)
   <- !mm::create_mission(search, 10, []); // scan
      +mm::mission_plan(search,L); // a list of waypoints
      !mm::run_mission(search).

+frl_charges(0)
   : my_number(N)
   <- .print(" No more Fire Retardant charges, going to recharge");
      !mm::create_mission(low_frl, 10, []); // Recharge Battery
      +mm::mission_plan(low_frl,[[0,0,N*5]]);
      !mm::run_mission(low_frl).

+!low_battery
   : my_number(N)
   <- +low_batt;
      .print(" Low Battery, going back to Recharge");
      !mm::create_mission(low_batt, 10, []); // Recharge Battery
      +mm::mission_plan(low_batt,[[0,0,N*5]]);
      !mm::run_mission(low_batt).

+mm::mission_state(low_batt,finished) 
   : my_number(N)
   <- .print(" Recharging Battery");
      .wait(10000);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","recharge_battery",N);
      .print(" Recharged!!");  //Still need to publish rechargeBattery topic
      -low_batt.
      

+mm::mission_state(low_frl,finished) 
   <- .print(" Recharging FRL");
      .wait(10000);
      .print(" Recharged!!");  //Still need to publish rechargeBattery topic
      -+frl_charges(4);
      !analyzeFire.

+!analyzeFire
   : fireSize(FS) & FS >0 & fire_pos(CX,CY) & std_altitude(Z) & my_number(N)
   <- .print(" Going back to fire!!");
      !mm::create_mission(goto_fire, 10, []); // gotofire
      +mm::mission_plan(goto_fire,[[CX,CY,Z+N]]);
      !mm::run_mission(goto_fire).

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
      !analyze_CNP.

+!analyze_CNP
   : fireSize(FS) & frl_charges(FRL) & FS > FRL
   <- !cnp( 2,help,(FS-FRL)).

+mm::mission_state(combat_fire,finished)   // Priority
   : fireSize(FS) & FS==0
   <- .print("Fire Extinguished").

+frl_charges(FRL)
   : fireSize(FS) & current_mission(combat_fire) & FS>FRL
   <- !cnp( 2,help,(FS-FRL)).

+mm::mission_state(combat_fire,finished) 
   : frl_charges(FRL) & FRL>=1
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","fightFire",FRL);
      -+frl_charges(FRL-1);
      .wait(1000);
      !mm::run_mission(combat_fire).

price(_Service,X,Y,R) :- 
   current_position(X, Y, CZ) & 
   frl_charges(R). //math.floor(R*20)+1.
// answer to Call For Proposal
@c1 +cfp(CNPId,Task)[source(A)]
   :  price(Task,X,Y,R)
   <- +proposal(CNPId,Task,X,Y,R); // remember my proposal
      .send(A,tell,propose(CNPId,X,Y,R)).

@r1 +accept_proposal(CNPId)[source(A)]
   :  proposal(CNPId,Task,X,Y,R) & fire_pos(CX,CY) & std_altitude(Z) & my_number(N)
   <- .print("My proposal '",R,"' was accepted for CNP ",CNPId, ", task ",Task," for agent ",A,"!");
      .print("Going to fire in : ",CX," , ",CY);  
      !mm::create_mission(goto_fire, 10, []); // gotofire
      +mm::mission_plan(goto_fire,[[CX,CY,Z+N]]);
      !mm::run_mission(goto_fire).
   
@r2 +reject_proposal(CNPId)
   <- .print("My proposal was not accepted for CNP ",CNPId, ".");
      -proposal(CNPId,_,_,_,_). // clear memory

+!cnp(Id,Task,TR)
   <- !call(Id,Task);
      !bids(Id,LO,TR);
      !result(Id,LO,TR).
+!call(Id,Task)
   : fire_pos(CX,CY)
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

+mm::mission_state(goto_fire,finished)  //goto fire finished
   : fire_pos(CX,CY) & std_altitude(Z) & my_number(N) & combat_traj(CT)
   <- .print("Go to fire finished!");
      !mm::create_mission(combat_fire, 10, [drop_when_interrupted]);
      +mm::mission_plan(combat_fire,CT);
      !mm::run_mission(combat_fire).

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
      !mm::create_mission(goto_land, 10, []); // gotofire
      +mm::mission_plan(goto_land,[[LAX,LAY,Z]]);
      !mm::run_mission(goto_land).

+!wait_for_others 
   <-.wait(1000);
      !wait_for_others.

+mm::mission_state(goto_land,finished) 
   <- .print(" Arrived at landing point, landing!");
       embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","land",[]).

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
   <- -current_mission(_);
      +current_mission(Id).

+!found_fire.
+!my_missions.
+!analyzeFire.
+!analyze_CNP.
