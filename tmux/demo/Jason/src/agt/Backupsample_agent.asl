mynumber(1).
traj([[-5,-5,5],[5,-5,5],[5,5,5],[-5,5,5]]).

!test.


+!test
   : traj(L)
   & mynumber(N)
   <- 
   
   // the action test_mrs_topic_action_full, executed below, considers all the parameters of the ros topic
   //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_full",[[0,[0,0],""],0,true,true,false,false,5.0,0.0,false,false,0.0,0.0,0.0,0.0,0.0,0.0,false,[[[0,0.5,5],0],[[2,-0.5,5],0],[[4,0.5,55],0]]] ).
   
   // the action test_mrs_topic_action_light, executed below, considers a subset of the parameters of the ros topic. In this case, these parameters are coordinates
   
   //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_light",[[-5,-5,5],[5,-5,5],[5,5,5],[-5,5,5]] ).
   
   //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_light",[[-10,-10,10],[10,-10,10],[10,10,10],[-10,10,10]] ).
   
   embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_light",[N,L] ).
   
   //embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("sample_roscore","test_mrs_topic_action_light",[[-5,-5,5],[5,-5,5]] ).
   //.wait(1000);
   //!test.
