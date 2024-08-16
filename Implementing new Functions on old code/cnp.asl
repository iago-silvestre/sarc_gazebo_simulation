//////////////// Initial beliefs
status("None").
world_area(100, 100, 0, 0).
num_of_uavs(4).
camera_range(5).
std_altitude(20.0).
std_heading(0.0).
land_radius(10.0).
currentwaypoint(0).
frl_charges(1).
temp_limit(70.5).
wind_limit(72.5).
diff(1).
cnp_limit(0).
fireLoc(24.5, -23.5).
landing_x(0.0).
landing_y(0.0).


current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(1) & uav1_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(2) & uav2_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(3) & uav3_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(4) & uav4_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(5) & uav3_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(6) & uav4_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).

near(X, Y) :- current_position(CX, CY, CZ)
              & diff(D)
              & math.abs(CX - X) <= D
              & math.abs(CY - Y) <= D.
my_number_string(S) :- my_number(N)
                       & .term2string(N, S).

+detect_fire_uav1(N) : my_number(N) <- !detected_fire(N).
+detect_fire_uav2(N) : my_number(N) <- !detected_fire(N).
+detect_fire_uav3(N) : my_number(N) <- !detected_fire(N).
+detect_fire_uav4(N) : my_number(N) <- !detected_fire(N).
+detect_fire_uav5(N) : my_number(N) <- !detected_fire(N).
+detect_fire_uav6(N) : my_number(N) <- !detected_fire(N).

+failure_uav1(N)<- !detected_failure.
+block(N) <- +failure.
+unblock(N) <- +unblocked.
+battery(B) : B<=30.0 & not(low_batt) <- !low_battery.

!start.

+!start
    <- .wait(5000);
      .print("Started!");
      !calculate_trajectory;
      !follow_trajectory(0).

+!goto_position(X, Y)
   : std_altitude(Z)
   <- -+status("goto_position");
      .print("going to fire position: ",X," ",Y);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","goto", [1, X, Y, 15.0, 0.0]).     

+!fightFire
   :   current_position(CX, CY, CZ)
   <- -+status("fighting_Fire");
      .print("Fighting Fire");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","fightFire",1);
      .wait(5000);
      !fightFire.
      
+failure
   <- .print("Suspending Trajectory!");
      -unblocked.
      
+unblocked
   <- .print("Resuming Trajectory!");
      .wait(2000);
      -failure.
      
+!hover
   <- -+status("hovering");
      .wait(1000);
      .print("hovering");
      !hover.
   
+!detected_failure
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","adf",N).
   

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
      !calculate_waypoints(1, []).

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

+!follow_trajectory(CW)
   :  waypoints_list_len(CW)
      & my_number(N)
   <- .broadcast(tell, finished_trajectory(N));
      +finished_trajectory(N);
      -+status("finished_trajectory");
      .print("finished_trajectory");
      !wait_for_others.

+!follow_trajectory(CW)
   :  waypoints_list(WL)
      & waypoints_list_len(Len)
      & CW < Len
   <- -+status("following_trajectory");
      .print("following_trajectory");
      .nth(CW, WL, [X, Y, Z]);
      !check_near(X, Y, Z, "waypoint");
      !follow_trajectory(CW+1).
      
+!wait_for_others
   :  my_number(N)
      & my_landing_position(LAX, LAY)
      & .count(finished_trajectory(_), C)
      & num_of_uavs(C)
   <- .print("All finished, going to land position");
      !goto_landing_position(LAX, LAY).

+!wait_for_others
   <- -+status("waiting");
      .print("Still waiting");
      .wait(1000);
      !wait_for_others.

+!goto_landing_position(X, Y)
   : std_altitude(Z)
   <- -+status("going_to_land_position");
      !check_near(X, Y, Z, "land position");
      !land.

+!land
   :  my_number_string(N)
   <- .print("Landing");
      -+status("landing");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1", "land", [N]).


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
	  !analyze_CNP;
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
   
+!goto_recharge_position
   <- !check_near(0, 0, 10, "recharge position").
   
+!low_battery
  : my_number(N) & .intend(I)
  <- +low_batt;
     .print(" Low Battery, going back to Recharge");
     !recharge_batt.
	 
+!recharge_batt
  <- .suspend(follow_trajectory);
     !goto_recharge_position;
     .print(" Recharging Battery");
     .wait(10000);
     embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","recharge_battery",N);
     .print(" Recharged!!"); 
     -low_batt;
	 .resume_batt.
	 
+!resume_batt
   : status("combat_fire") & fire_Size(FS) & FS>0
   <- !goto_fire_position(FX,FY,10);
	  .resume(combat_fire).
	  
+!resume_batt
   : status("follow_trajectory")
   <- .resume(follow_trajectory).	
	 
+!combat_fire
   : current_position(CX, CY, CZ)
   <- .wait(10000);
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","drop",[CX, CY, 8.0]);
      +fire_extinguished;
      .resume(wait_for_others);
      .print("Fire extinguished. Resuming waiting").

+!recharge_frl
	<- !goto_recharge_position;
	   .print(" Recharging FRL");
	   .wait(10000);
	   -+frl_charges(4);
	   .print(" Recharged!!");
	   !analyze_fire.
	   
+!analyze_fire
	: fire_Size(FS) & FS>0 & fire_pos(FX,FY)
	<- !goto_fire_position(FX,FY,10);
	   .resume(combat_fire).

+!analyze_fire
	: fire_Size(0)
	<- .resume(follow_trajectory).
      
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

+!analyze_CNP
  : fireSize(FS) & frl_charges(FRL) & FS > FRL
  <- !cnp( 2,help,(FS-FRL)).

+frl_charges(FRL)
  : fireSize(FS) & status(combat_fire) & FS>FRL
  <- !cnp( 2,help,(FS-FRL)).
	  
price(_Service,X,Y,R) :- 
  current_position(X, Y, CZ) & 
  frl_charges(R). //math.floor(R*20)+1.
answer to Call For Proposal
@c1 +cfp(CNPId,Task)[source(A)]
  :  price(Task,X,Y,R)
  <- +proposal(CNPId,Task,X,Y,R); // remember my proposal
     .send(A,tell,propose(CNPId,X,Y,R)).

@r1 +accept_proposal(CNPId)[source(A)]
  :  proposal(CNPId,Task,X,Y,R) & fire_pos(CX,CY) & std_altitude(Z) & my_number(N)
  <- .print("My proposal '",R,"' was accepted for CNP ",CNPId, ", task ",Task," for agent ",A,"!");
     .print("Going to fire in : ",CX," , ",CY);  
     .suspend(follow_trajectory);
	 !goto_fire_position;
	 !combat_fire.
   
@r2 +reject_proposal(CNPId)
  <- .print("My proposal was not accepted for CNP ",CNPId, ".");
     -proposal(CNPId,_,_,_,_). // clear memory

+!cnp(Id,Task,TR)
  <- !call(Id,Task);
     !bids(Id,LO,TR);
     !result(Id,LO,TR).
	  
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
      .print("New list of offers ",LOS);
     !result(CNPId,LOS,ND).
+!result(CNPId,[offer(_,_,_,LAg)|T],RT) // announce to others
  <- .send(LAg,tell,reject_proposal(CNPId));
     !result(CNPId,T,RT).

// /* Initial beliefs and rules */

all_proposals_received(CNPId,NP) :-              // NP = number of participants
    .count(propose(CNPId,_,_,_)[source(_)], NO) &   // number of proposes received
    .count(refuse(CNPId)[source(_)], NR) &      // number of refusals received
    NP = NO + NR.	  

+!check_near(X, Y, Z, S)
   :  near(X, Y)
   <- .print("Arrived at ", S).

+!check_near(X, Y, Z, S)
   :  my_number(N)
      & std_heading(Heading)
      & not failure
   <- embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","goto", [N, X, Y, Z, Heading]);
      .wait(100);
      !check_near(X, Y, Z, S).
    
+!check_near(X, Y, Z, S)
   :  my_number_string(N)
      & std_heading(Heading)
      & failure
  <- .wait(500);
     !check_near(X, Y, Z, S).

+!detected_failure(_).
+!detected_fire(_).
+!found_fire(_, _, _).
+!analyze_CNP.