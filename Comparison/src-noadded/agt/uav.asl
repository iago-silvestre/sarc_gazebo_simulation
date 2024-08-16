{ include("mission-management2.asl", mm) }

current_mission("None").
status("None").
world_area(100, 100, 0, 0).
num_of_uavs(6).
nb_participants(5).
camera_range(5).
std_altitude(6.25).
std_heading(0.0).
land_radius(10.0).
wind_speed(-20.0).
landing_x(0.0).
landing_y(0.0).
fire_pos(0.0,0.0).

current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(1) & uav1_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(2) & uav2_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(3) & uav3_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(4) & uav4_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(5) & uav5_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
current_position(CX, CY, CZ) :- my_frame_id(Frame_id) & my_number(6) & uav6_ground_truth(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(Frame_id)),child_frame_id(CFI),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(OX),y((OY)),z((OZ)),w((OW)))),covariance(CV)),twist(twist(linear(x(LX),y(LY),z((LZ))),angular(x(AX),y((AY)),z((AZ)))),covariance(CV2))).
combat_traj(CT) :- wind_speed(WS) & WS >=0.0 & fire_pos(CX,CY) & std_altitude(Z)  & my_number(N)
                 & CT= [[CX-2,CY+2,Z+N],[CX+2,CY+2,Z+N],[CX+2,CY-2,Z+N],[CX-2,CY-2,Z+N]].
combat_traj(CT) :- wind_speed(WS) & WS < 0.0 & fire_pos(CX,CY) & std_altitude(Z)  & my_number(N) 
                 & CT= [[CX-2,CY-2,Z+N],[CX+2,CY-2,Z+N],[CX+2,CY+2,Z+N],[CX-2,CY+2,Z+N]].

my_ap(AP) :- my_number(N)
            & .term2string(N, S) & .concat("autopilot",S,AP).

distance(X,Y,D) :- current_position(CX, CY, CZ) & D=math.sqrt( (CX-X)**2 + (CY-Y)**2 ).

+fire_detection(N) : N>=22000 <- !found_fire.
!start.
+!start
   : my_ap(AP) & my_number(N) & combat_traj(CT)
    <- .wait(2000);
      +mm::my_ap(AP);
      .print("Started!",CT);
      !calculate_trajectory;
      !my_missions.

+!my_missions
   :  waypoints_list(L) & my_number(N) 
   <- !mm::create_mission(search, 10, []); 
      +mm::mission_plan(search,L); 
      !mm::run_mission(search).

+!my_missions
   :  waypoints_list(L) & my_number(N) & not (N==1)
   <- !mm::create_mission(search, 10, []); 
      +mm::mission_plan(search,L); 
      !mm::run_mission(search).

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
      !mm::create_mission(goto_land, 10, []); 
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

+mm::mission_state(Id,S) 
   <- .print("Mission ",Id," state is ",S).

+mm::current_mission(Id)
   <- -current_mission(_);
      +current_mission(Id).

+!found_fire.
+!my_missions.
+!analyzeFire.
