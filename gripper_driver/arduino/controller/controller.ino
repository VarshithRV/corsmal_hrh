/*
 * rosserial driver for gripper
 */

 #include <ros.h>
 #include <std_msgs/UInt8MultiArray.h>
 #include <std_msgs/Float32.h>
 
 ros::NodeHandle  nh;
 
 // setpoint contains incoming message
 std_msgs::UInt8MultiArray sticky_msg_gripper1;
 std_msgs::UInt8MultiArray setpoint_gripper1;
 int flag1=0;
 std_msgs::UInt8MultiArray sticky_msg_gripper2;
 std_msgs::UInt8MultiArray setpoint_gripper2;
 int flag2=0;
 std_msgs::Float32 debug;

 void gripper1_messageCb( const std_msgs::UInt8MultiArray& msg){
   sticky_msg_gripper1 = msg;
   if(flag1==0)
   {
    flag1 = 1;
    }
   else
   {
    flag1 = 1;
    }
 }

 void gripper2_messageCb( const std_msgs::UInt8MultiArray& msg){
   sticky_msg_gripper2 = msg;
   if(flag2==0)
   {
    flag2 = 1;
    }
   else
   {
    flag2 = 1;
    }
 }
 
 ros::Subscriber<std_msgs::UInt8MultiArray> sub_gripper1("/gripper1_setpoint", gripper1_messageCb );
 ros::Publisher gripper1_state("/gripper1_state", &setpoint_gripper1);

 ros::Subscriber<std_msgs::UInt8MultiArray> sub_gripper2("/gripper2_setpoint", gripper2_messageCb );
 ros::Publisher gripper2_state("/gripper2_state", &setpoint_gripper2);

 ros::Publisher debugger("/debug", &debug);

 void setup()
 {
   nh.initNode();
   nh.advertise(gripper1_state);
   nh.subscribe(sub_gripper1);
   nh.advertise(gripper2_state);
   nh.subscribe(sub_gripper2);
   nh.advertise(debugger);
   // configure pins for gripper 1
    pinMode(22,OUTPUT);
    pinMode(24,OUTPUT);
    pinMode(26,OUTPUT);
    pinMode(28,OUTPUT);
    pinMode(30,OUTPUT);
    pinMode(32,OUTPUT);
    pinMode(34,OUTPUT);
    pinMode(36,OUTPUT);
    pinMode(38,OUTPUT);
    pinMode(40,OUTPUT);
    pinMode(42,OUTPUT);
    pinMode(44,OUTPUT);
    pinMode(46,OUTPUT);
    // configure poins for gripper 2
    pinMode(23,OUTPUT);
    pinMode(25,OUTPUT);
    pinMode(27,OUTPUT);
    pinMode(29,OUTPUT);
    pinMode(31,OUTPUT);
    pinMode(33,OUTPUT);
    pinMode(35,OUTPUT);
    pinMode(37,OUTPUT);
    pinMode(39,OUTPUT);
    pinMode(41,OUTPUT);
    pinMode(43,OUTPUT);
    pinMode(45,OUTPUT);
    pinMode(47,OUTPUT);
    
 }
 
 void loop()
 {
   

   if(flag1==1)
   {
    setpoint_gripper1 = sticky_msg_gripper1;
    gripper1_state.publish( &setpoint_gripper1 );
    
    // write for gripper 1, even pins from 22-46
    digitalWrite(26,setpoint_gripper1.data[0]);
    digitalWrite(44,setpoint_gripper1.data[1]);
    digitalWrite(22,setpoint_gripper1.data[2]);
    digitalWrite(24,setpoint_gripper1.data[3]);
    digitalWrite(28,setpoint_gripper1.data[4]);
    digitalWrite(30,setpoint_gripper1.data[5]);
    digitalWrite(32,setpoint_gripper1.data[6]);
    digitalWrite(34,setpoint_gripper1.data[7]);
    digitalWrite(36,setpoint_gripper1.data[8]);
    digitalWrite(38,setpoint_gripper1.data[9]);
    digitalWrite(40,setpoint_gripper1.data[10]);
    digitalWrite(42,setpoint_gripper1.data[11]);
    digitalWrite(46,setpoint_gripper1.data[12]);

    // write for gripper 2, odd pins from 23 to 47
    

//    debug = setpoint_gripper1[0];
//    debugger.publish( &debug ); 
    }

    if(flag2==1)
   {
    setpoint_gripper2 = sticky_msg_gripper2;
    gripper2_state.publish( &setpoint_gripper2 );
    
    // write for gripper 1, odd pins from 23-47
    digitalWrite(23,setpoint_gripper2.data[0]);
    digitalWrite(25,setpoint_gripper2.data[1]);
    digitalWrite(27,setpoint_gripper2.data[2]);
    digitalWrite(29,setpoint_gripper2.data[3]);
    digitalWrite(31,setpoint_gripper2.data[4]);
    digitalWrite(33,setpoint_gripper2.data[5]);
    digitalWrite(35,setpoint_gripper2.data[6]);
    digitalWrite(37,setpoint_gripper2.data[7]);
    digitalWrite(39,setpoint_gripper2.data[8]);
    digitalWrite(41,setpoint_gripper2.data[9]);
    digitalWrite(43,setpoint_gripper2.data[10]);
    digitalWrite(45,setpoint_gripper2.data[11]);
    digitalWrite(47,setpoint_gripper2.data[12]);

    

//    debugger.publish( &debug ); 
    }
    
   nh.spinOnce();
   delay(50);
 }
 
