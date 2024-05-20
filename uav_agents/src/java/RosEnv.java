import jason.asSyntax.*;
import jason.environment.*;
import java.util.logging.*;

import embedded.mas.bridges.ros.ros.RosBridge;
import embedded.mas.bridges.ros.ros.RosListenDelegate;
import embedded.mas.bridges.ros.ros.Publisher;
import embedded.mas.bridges.ros.ros.msgs.std_msgs.PrimitiveMsg;
import embedded.mas.bridges.ros.ros.tools.MessageUnpacker;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import static embedded.mas.bridges.jacamo.Utils.jsonToPredArguments;

// import ros.Publisher;
// import ros.RosBridge;
// import ros.RosListenDelegate;
// import ros.SubscriptionRequestMsg;
// import ros.msgs.geometry_msgs.Twist;
// import ros.msgs.geometry_msgs.Vector3;
// import ros.msgs.std_msgs.PrimitiveMsg;
// import ros.msgs.std_msgs.Int32;
// import ros.msgs.ariac_msgs.Snapshot;
// import ros.msgs.ariac_msgs.TimedPose;
// import ros.msgs.move_base_msgs.MoveBaseActionResult;
// import ros.tools.MessageUnpacker;
// import com.fasterxml.jackson.databind.JsonNode;

public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("firefight_env."+RosEnv.class.getName());
    
    int cont = 0;

    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
        bridge.connect("ws://localhost:9090", true);
        logger.info("Firefight Environment started: connection with ROS established.");
    } //end init() - where subscriptions are

    @Override
    public boolean executeAction(String agName, Structure act) {
        if (act.getFunctor().equals("critReac0")) {
            // If has parameter, do as follows:
            //NumberTerm lx = (NumberTerm) act.getTerm(0);
            try{
                 //failSafe(lx.solve());
                 failSafe();
                 //addPercept(Literal.parseLiteral("failure"));
                 //addPercept("uav1",Literal.parseLiteral("failure"));
            } catch(Exception e) {
             e.printStackTrace();
            }
            // else (no parameter), do as bellow:
            //failSafe();
        }
        else {
            logger.info("PROBLEM: requested: "+act.getFunctor()+", but not implemented!");
        }
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }
    
    public void failSafe(double dPar) {
        Publisher pub = new Publisher("/agent_detected_failure_uav1","std_msgs/String",bridge); //"/current_time", "std_msgs/String", bridge);       
        pub.publish(new PrimitiveMsg<Double>(dPar)); //here goes the published parameters (arguments)
    }
    
    public void failSafe() {
        Publisher pub = new Publisher("/agent_detected_failure_uav1","std_msgs/String",bridge); //"/current_time", "std_msgs/String", bridge);       
        pub.publish(new PrimitiveMsg<String>("1")); //here goes the published parameters (arguments)
    }

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
 }
