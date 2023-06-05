/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: David Portugal (2011-2014), and Luca Iocchi (2014-2016)
*********************************************************************/

#include <sstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h> //to get pkg path
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include "patrolling_sim/anomaly_service.h"

#include "PatrolAgent.h"

using namespace std;

#define DELTA_TIME_SEQUENTIAL_START 15
#define SIMULATE_FOREVER true //WARNING: Set this to false, if you want a finishing condition.

const std::string PS_path = ros::package::getPath("patrolling_sim"); 	//D.Portugal => get pkg path

double last_pos_log = 60.0;
FILE *positionstimecsvfile;


void PatrolAgent::init(int argc, char** argv) {
        /*
            argv[0]=/.../patrolling_sim/bin/GBS
            argv[1]=__name:=XXXXXX
            argv[2]=grid
            argv[3]=ID_ROBOT
        */
    
    srand ( time(NULL) );
    
    //More than One robot (ID between 0 and 99)
    if ( atoi(argv[3])>NUM_MAX_ROBOTS || atoi(argv[3])<-1 ){
        ROS_INFO("The Robot's ID must be an integer number between 0 an 99"); //max 100 robots 
        return;
    }else{
        ID_ROBOT = atoi(argv[3]); 
        //printf("ID_ROBOT = %d\n",ID_ROBOT); //-1 for 1 robot without prefix (robot_0)
    }
    
    /** D.Portugal: needed in case you "rosrun" from another folder **/     
    chdir(PS_path.c_str());
                
    mapname = string(argv[2]);
    graph_file = "maps/"+mapname+"/"+mapname+".graph";
    
    //Check Graph Dimension:
    dimension = GetGraphDimension(graph_file.c_str());
    
    //Create Structure to save the Graph Info;
    vertex_web = new vertex[dimension];
    //Get the Graph info from the Graph File
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());

    //Initialise belief states
    belief_states.resize(dimension);
    second_agent_belief_states.resize(dimension);
    std::fill(belief_states.begin(), belief_states.end(), 0.5);
    uint nedges = GetNumberEdges(vertex_web,dimension);
    
    printf("Loaded graph %s with %d nodes and %d edges\n",mapname.c_str(),dimension,nedges);

#if 0
    /* Output Graph Data */   
    for (i=0;i<dimension;i++){
        printf ("ID= %u\n", vertex_web[i].id);
        printf ("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf ("#Neigh= %u\n", vertex_web[i].num_neigh);
        
        for (j=0;j<vertex_web[i].num_neigh; j++){
        printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }
        
        printf("\n");   
    }
#endif
      
    interference = false;
    crashed = false;
    ResendGoal = false;
    goal_complete = true;
    last_interference = 0;
    goal_canceled_by_user = false;
    aborted_count = 0;
    resend_goal_count = 0;
    communication_delay = 0.0;
    lost_message_rate = 0.0;
    goal_reached_wait = 0.0;
    communication_distance = 4;
    communication_period = 5;
    /* Define Starting Vertex/Position (Launch File Parameters) */
    ros::init(argc, argv, "patrol_agent");  // will be replaced by __name:=XXXXXX
    ros::NodeHandle nh;

    // Set position log save file
    if (ID_ROBOT == 0) {

        // Get experiment name from parameter server
        string expname;
        nh.getParam("/exp_name", expname);

        string positionstimecsvfilename = expname;
        positionstimecsvfilename.append("_position_log.csv");
        positionstimecsvfile = fopen (positionstimecsvfilename.c_str(),"a");
    }

    // wait a random time (avoid conflicts with other robots starting at the same time...)
    double r = 3.0 * ((rand() % 1000)/1000.0);
    ros::Duration wait(r); // seconds
    wait.sleep();
    
    double initial_x, initial_y;
    std::vector<double> list;
    nh.getParam("initial_pos", list);
    
    if (list.empty()){
     ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
     ros::shutdown();
     exit(-1);
    }
       
    int value = ID_ROBOT;
    if (value == -1){value = 0;}
    
    initial_x = list[2*value];
    initial_y = list[2*value+1];
    
    //   printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
    current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
    //   printf("initial vertex = %d\n\n",current_vertex);  
    
    
    //instantaneous idleness and last visit initialized with zeros:
    instantaneous_idleness = new double[dimension];
    last_visit = new double[dimension];
    for(size_t i=0; i<dimension; i++){ 
        instantaneous_idleness[i]= 0.0; 
        last_visit[i]= 0.0; 
        
        if (i==current_vertex){
            last_visit[i]= 0.1; //Avoids getting back at the initial vertex
        }
        //ROS_INFO("last_visit[%d]=%f", i, last_visit[i]);
    }
    //Publish data from "odom" to positions node
    positions_pub = nh.advertise<nav_msgs::Odometry>("positions", 1); //only concerned about the most recent
        
    //Subscribe positions from other robots
    positions_sub = nh.subscribe<nav_msgs::Odometry>("positions", 10, boost::bind(&PatrolAgent::positionsCB, this, _1));  
    
    char string1[40];
    char string2[40];
    char string3[40];
    
    if(ID_ROBOT==-1){ 
        strcpy (string1,"odom"); //string = "odom"
        strcpy (string2,"cmd_vel"); //string = "cmd_vel"
        strcpy (string3,"belief_state"); //string = "belief_state"
        TEAMSIZE = 1;
    }else{ 
        sprintf(string1,"robot_%d/odom",ID_ROBOT);
        sprintf(string2,"robot_%d/cmd_vel",ID_ROBOT);
        sprintf(string3, "robot_%d/belief_state",ID_ROBOT);
        TEAMSIZE = ID_ROBOT + 1;
    }   

    /* Set up listener for global coordinates of robots */
    listener = new tf::TransformListener();

    //Cmd_vel to backup:
    cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>(string2, 1);

    //Get TEAMSIZE from parameter server, like a normal person
    nh.getParam("TEAMSIZE", TEAMSIZE);
    nh.getParam("communication_distance", communication_distance);
    nh.getParam("communication_period", communication_period);
    //Initialises all agents with current time as last communication time
    communication_history.resize(TEAMSIZE);

    for(size_t i=0; i<TEAMSIZE; i++) {
        communication_history[i] = ros::Time::now().toSec();
    }

    //creates vector of topic names -- includes robot's OWN ID, we want to subscribe to it to get updates from other agents that initiate the pairwise comparison
    vector<string> belief_topics_vector(32, "");
    char topic_name[40];

    for(int i=0; i < TEAMSIZE; i++){
        sprintf(topic_name, "robot_%d/belief_state", i);
        belief_topics_vector[i] = topic_name;
    }

    //Publisher for belief states
    belief_publisher.resize(TEAMSIZE);

    for(int i=0; i<TEAMSIZE; i++){
        belief_publisher[i] = nh.advertise<std_msgs::Float32MultiArray>(belief_topics_vector[i], 1, true);
    }

    sprintf(topic_name, "robot_%d/belief_state", ID_ROBOT);

    //Specific callback function for agent's own topic
    personal_belief_subscriber = nh.subscribe<std_msgs::Float32MultiArray>(topic_name, 1, boost::bind(&PatrolAgent::personal_belief_topic_CB, this, _1));

    //Service client for anomaly node
    anomaly_client = nh.serviceClient<patrolling_sim::anomaly_service>("anomaly_service");

    // Subscrever para obter dados de "odom" do robot corrente
    // Subscribe to get "odom" data from current robot
    odom_sub = nh.subscribe<nav_msgs::Odometry>(string1, 1, boost::bind(&PatrolAgent::odomCB, this, _1)); //size of the buffer = 1 (?)
    
    ros::spinOnce(); 
    
    //Publicar dados para "results"
    results_pub = nh.advertise<std_msgs::Int16MultiArray>("results", 100);
    // results_sub = nh.subscribe("results", 10, resultsCB); //Subscrever "results" vindo dos robots
    results_sub = nh.subscribe<std_msgs::Int16MultiArray>("results", 100, boost::bind(&PatrolAgent::resultsCB, this, _1) ); //Subscrever "results" vindo dos robots

    // last time comm delay has been applied
    last_communication_delay_time = ros::Time::now().toSec();   

    readParams();
}
    
void PatrolAgent::ready() {
    
    char move_string[40];
    
    /* Define Goal */
    if(ID_ROBOT==-1){ 
        strcpy (move_string,"move_base"); //string = "move_base
    }else{
        sprintf(move_string,"robot_%d/move_base",ID_ROBOT);
    }
    
    ac = new MoveBaseClient(move_string, true);
    //wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Connected with move_base action server");    
    
    initialize_node(); //announce that agent is alive
    
    ros::Rate loop_rate(1); //1 sec
    
    /* Wait until all nodes are ready.. */
    while(initialize){
        ros::spinOnce();
        loop_rate.sleep();
    }    

}


void PatrolAgent::readParams() {

    if (! ros::param::get("/goal_reached_wait", goal_reached_wait)) {
      //goal_reached_wait = 0.0;
      ROS_WARN("Cannot read parameter /goal_reached_wait. Using default value!");
      //ros::param::set("/goal_reached_wait", goal_reached_wait);
    }

    if (! ros::param::get("/communication_delay", communication_delay)) {
      //communication_delay = 0.0;
      ROS_WARN("Cannot read parameter /communication_delay. Using default value!");
      //ros::param::set("/communication_delay", communication_delay);
    } 

    if (! ros::param::get("/lost_message_rate", lost_message_rate)) {
      //lost_message_rate = 0.0;
      ROS_WARN("Cannot read parameter /lost_message_rate. Using default value!");
      //ros::param::set("/lost_message_rate", lost_message_rate);
    }

    if (! ros::param::get("/initial_positions", initial_positions)) {
      //initial_positions = "default";
      ROS_WARN("Cannot read parameter /initial_positions. Using default value '%s'!", initial_positions.c_str());
      //ros::param::set("/initial_pos", initial_positions);
    }

}

void PatrolAgent::run() {
    // get ready
    ready();
    
    //initially clear the costmap (to make sure the robot is not trapped):
    std_srvs::Empty srv;
    std::string mb_string;
     
     if (ID_ROBOT>-1){
             std::ostringstream id_string;
             id_string << ID_ROBOT;
             mb_string = "robot_" + id_string.str() + "/";
    }
    mb_string += "move_base/clear_costmaps";
    
    if (ros::service::call(mb_string.c_str(), srv)){
    //if (ros::service::call("move_base/clear_costmaps", srv)){
        ROS_INFO("Costmap correctly cleared before patrolling task.");
    }else{
        ROS_WARN("Was not able to clear costmap (%s) before patrolling...", mb_string.c_str());
    }
    
    // Asynch spinner (non-blocking)
    ros::AsyncSpinner spinner(2); // Use n threads
    spinner.start();
//     ros::waitForShutdown();

    /* Run Algorithm */ 
    
    ros::Rate loop_rate(30); //0.033 seconds or 30Hz
    
    while(ros::ok()){
        
        if (goal_complete) {
            onGoalComplete();  // can be redefined
            call_anomaly_service(next_vertex);
            resend_goal_count=0;
        }
        else if (not crashed) { // goal not complete (active)
            if (interference) {
                do_interference_behavior();
            }    
            
            if (ResendGoal) {
                //Send the goal to the robot (Global Map)
                if (resend_goal_count<3) {
                    resend_goal_count++;
                    ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
                    sendGoal(next_vertex);
                }
                else {
                    resend_goal_count=0;
                    onGoalNotComplete();
                }
                ResendGoal = false; //para nao voltar a entrar (envia goal so uma vez)
            }
            
            processEvents();
            
            if (end_simulation) {
                return;
            }   
        
        } // if (goal_complete)
        if(check_comparison_range()){
            ROS_INFO("Comparison was performed! yay");
        }
		loop_rate.sleep();
    } // while ros.ok    
}


void PatrolAgent::onGoalComplete()
{
    if(next_vertex>-1) {
        //Update Idleness Table:
        update_idleness();
        current_vertex = next_vertex;       
    }
    
    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex = compute_next_vertex();
    //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    /** SEND GOAL (REACHED) AND INTENTION **/
    send_goal_reached(); // Send TARGET to monitor
    send_results();  // Algorithm specific function

    //Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(next_vertex);  // send to move_base
    
    goal_complete = false;    
}

void PatrolAgent::onGoalNotComplete()
{   
    int prev_vertex = next_vertex;
    
    ROS_INFO("Goal not complete - From vertex %d to vertex %d\n", current_vertex, next_vertex);   
    
    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex = compute_next_vertex();
    //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    // Look for a random adjacent vertex different from the previous one
    int random_cnt=0;
    while (next_vertex == prev_vertex && random_cnt++<10) {
        int num_neighs = vertex_web[current_vertex].num_neigh;
        int i = rand() % num_neighs;
        next_vertex = vertex_web[current_vertex].id_neigh[i];
        ROS_INFO("Choosing another random vertex %d\n", next_vertex);
    }
    
    // Look for any random vertex different from the previous one
    while (next_vertex == prev_vertex && next_vertex == current_vertex) {
        int i = rand() % dimension;
        next_vertex = i;
        ROS_INFO("Choosing another random vertex %d\n", next_vertex);
    }

    //Send the goal to the robot (Global Map)
    ROS_INFO("Re-Sending NEW goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(next_vertex);  // send to move_base
    
    goal_complete = false;    
}


void PatrolAgent::processEvents() {
    
}

void PatrolAgent::update_idleness() {
    double now = ros::Time::now().toSec();
        
    for(size_t i=0; i<dimension; i++){
        if ((int)i == next_vertex){
            last_visit[i] = now;    
        }
        instantaneous_idleness[i] = now - last_visit[i];           
	
	//Show Idleness Table:
	//ROS_INFO("idleness[%u] = %f",i,instantaneous_idleness[i]);
    }
}

void PatrolAgent::initialize_node (){ //ID,msg_type,1
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    ROS_INFO("Initialize Node: Robot %d",value); 
    
    std_msgs::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(INITIALIZE_MSG_TYPE);
    msg.data.push_back(1);  // Robot initialized
    
    int count = 0;
    
    //ATENÇÃO ao PUBLICADOR!
    ros::Rate loop_rate(0.5); //meio segundo
    
    while (count<3){ //send activation msg 3times
        results_pub.publish(msg);
        //ROS_INFO("publiquei msg: %s\n", msg.data.c_str());
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
}

void PatrolAgent::getRobotPose(int robotid, float &x, float &y, float &theta) {
    
    if (listener==NULL) {
        ROS_ERROR("TF listener null");
        return;
    }
    
    std::stringstream ss; ss << "robot_" << robotid;
    std::string robotname = ss.str();
    std::string sframe = "/map";                //Patch David Portugal: Remember that the global map frame is "/map"
    std::string dframe;
    if(ID_ROBOT>-1){
        dframe = "/" + robotname + "/base_link";
    }else{
        dframe = "/base_link";
    }
    
    tf::StampedTransform transform;

    try {
        listener->waitForTransform(sframe, dframe, ros::Time(0), ros::Duration(3));
        listener->lookupTransform(sframe, dframe, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("Cannot transform from %s to %s\n",sframe.c_str(),dframe.c_str());
        ROS_ERROR("%s", ex.what());
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    theta = tf::getYaw(transform.getRotation());
    // printf("Robot %d pose : %.1f %.1f \n",robotid,x,y);
}

void PatrolAgent::odomCB(const nav_msgs::Odometry::ConstPtr& msg) { //colocar propria posicao na tabela
    
//  printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
    int idx = ID_ROBOT;
    
    if (ID_ROBOT<=-1){
        idx = 0;
    }
    
    float x,y,th;
    getRobotPose(idx,x,y,th);
    
    xPos[idx]=x; // msg->pose.pose.position.x;
    yPos[idx]=y; // msg->pose.pose.position.y;
    
//  printf("Posicao colocada em Pos[%d]\n",idx);
}



void PatrolAgent::sendGoal(int next_vertex) 
{
    goal_canceled_by_user = false;
    
    double target_x = vertex_web[next_vertex].x, 
           target_y = vertex_web[next_vertex].y;
    
    //Define Goal:
    move_base_msgs::MoveBaseGoal goal;
    //Send the goal to the robot (Global Map)
    geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);     
    goal.target_pose.header.frame_id = "map"; 
    goal.target_pose.header.stamp = ros::Time::now();    
    goal.target_pose.pose.position.x = target_x; // vertex_web[current_vertex].x;
    goal.target_pose.pose.position.y = target_y; // vertex_web[current_vertex].y;  
    goal.target_pose.pose.orientation = angle_quat; //doesn't matter really.
    ac->sendGoal(goal, boost::bind(&PatrolAgent::goalDoneCallback, this, _1, _2), boost::bind(&PatrolAgent::goalActiveCallback,this), boost::bind(&PatrolAgent::goalFeedbackCallback, this,_1));  
}

void PatrolAgent::cancelGoal() 
{
    goal_canceled_by_user = true;
    ac->cancelAllGoals();
}


void PatrolAgent::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){ //goal terminado (completo ou cancelado)
//  ROS_INFO("Goal is complete (suceeded, aborted or cancelled).");
    // If the goal succeeded send a new one!
    //if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
    // If it was aborted time to back up!
    //if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;    
    
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Goal reached ... WAITING %.2f sec",goal_reached_wait);
        ros::Duration delay(goal_reached_wait); // wait after goal is reached
        delay.sleep();
        ROS_INFO("Goal reached ... DONE");
        goal_complete = true;
    }else{
        aborted_count++;
        ROS_INFO("CANCELLED or ABORTED... %d",aborted_count);   //tentar voltar a enviar goal..
        if (!goal_canceled_by_user) {
            ROS_INFO("Goal not cancelled by the interference...");

            //ROS_INFO("Backup");
            backup();

            ROS_INFO("Clear costmap!");

            char srvname[80];
            
            if(ID_ROBOT<=-1){
                sprintf(srvname,"/move_base/clear_costmaps");
            }else{
                sprintf(srvname,"/robot_%d/move_base/clear_costmaps",ID_ROBOT);
            }
            
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(srvname);
            std_srvs::Empty srv;
            if (client.call(srv)) {
                ROS_INFO("Costmaps cleared.\n");
            }
            else {
                ROS_ERROR("Failed to call service move_base/clear_costmaps");
            }

            ROS_INFO("Resend Goal!");
            ResendGoal = true;
        }
    }
}

void PatrolAgent::goalActiveCallback(){  //enquanto o robot esta a andar para o goal...
    goal_complete = false;
//      ROS_INFO("Goal is active.");
}

void PatrolAgent::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){    //publicar posições

    send_positions();
    
    int value = ID_ROBOT;
    if (value==-1){ value = 0;}
    interference = check_interference(value);
    crashed = check_collision(value);    
}

void PatrolAgent::send_goal_reached() {
    
    int value = ID_ROBOT;
    if (value==-1){ value = 0;}
    
    // [ID,msg_type,vertex,intention,0]
    std_msgs::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(TARGET_REACHED_MSG_TYPE);
    msg.data.push_back(current_vertex);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?
    
    results_pub.publish(msg);   
    ros::spinOnce();  
}

bool PatrolAgent::check_interference (int robot_id){ //verificar se os robots estao proximos
    
    int i;
    double dist_quad;
    
    if (ros::Time::now().toSec()-last_interference<10)  // seconds
        return false; // false if within 10 seconds from the last one
    
    /* Poderei usar TEAMSIZE para afinar */
    for (i=0; i<robot_id; i++){ //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)
        
        dist_quad = (xPos[i] - xPos[robot_id])*(xPos[i] - xPos[robot_id]) + (yPos[i] - yPos[robot_id])*(yPos[i] - yPos[robot_id]);
        
        if (dist_quad <= INTERFERENCE_DISTANCE*INTERFERENCE_DISTANCE){    //robots are ... meter or less apart
//          ROS_INFO("Feedback: Robots are close. INTERFERENCE! Dist_Quad = %f", dist_quad);
            last_interference = ros::Time::now().toSec();
            return true;
        }       
    }
    return false;
    
}

bool PatrolAgent::check_collision (int robot_id){ //verificar se os robots estao proximos
    
    int i;
    double dist_quad;
    
    /* Poderei usar TEAMSIZE para afinar */
    for (i=0; i<TEAMSIZE; i++){ //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)

        if (i != robot_id) {
        
            dist_quad = (xPos[i] - xPos[robot_id])*(xPos[i] - xPos[robot_id]) + (yPos[i] - yPos[robot_id])*(yPos[i] - yPos[robot_id]);
            
            // if (dist_quad <= 0.25*0.25){    //robots are ... meter or less apart
            //     cancelGoal();
            //     ROS_INFO("Robot stopped");
            //     return true;
            // }    
        }   
    }
    return false; 
}


void PatrolAgent::backup(){
    
    ros::Rate loop_rate(100); // 100Hz
    
    int backUpCounter = 0;
    while (backUpCounter<=100){
    
      if(backUpCounter==0){
          ROS_INFO("The wall is too close! I need to do some backing up...");
          // Move the robot back...
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = -0.5;
          cmd_vel.angular.z = 1.0;
          cmd_vel_pub.publish(cmd_vel);
      }
              
      // if(backUpCounter==50){
      //     // Turn the robot around...
      //     geometry_msgs::Twist cmd_vel;
      //     cmd_vel.linear.x = 0.0;
      //     cmd_vel.angular.z = 0.5;
      //     cmd_vel_pub.publish(cmd_vel);
      // }
              
      if(backUpCounter==25){
          // Stop the robot...
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          cmd_vel_pub.publish(cmd_vel);
              
          // ROS_INFO("Done backing up, now on with my life!");      
      }

      ros::spinOnce();
      loop_rate.sleep();
      backUpCounter++;
    
    }
}

void PatrolAgent::do_interference_behavior()
{
    ROS_INFO("Interference detected! Executing interference behavior...\n");   
    send_interference();  // send interference to monitor for counting
    
#if 1
    // Stop the robot..         
    cancelGoal();
    ROS_INFO("Robot stopped");
    ros::Duration delay(0.5); // seconds
    delay.sleep();
    ResendGoal = true;
#else    
    //get own "odom" positions...
    ros::spinOnce();        
                
    //Waiting until conflict is solved...
    int value = ID_ROBOT;
    if (value == -1){ value = 0;}
    while(interference){
        interference = check_interference(value);
        if (goal_complete || ResendGoal){
            interference = false;
        }
    }
#endif
}




// ROBOT-ROBOT COMMUNICATION



void PatrolAgent::send_positions()
{
    //Publish Position to common node:
    nav_msgs::Odometry msg;

    int idx = ID_ROBOT;

    if (ID_ROBOT <= -1){
        msg.header.frame_id = "map";    //identificador do robot q publicou
        idx = 0;
    }else{
        char string[20];
        sprintf(string,"robot_%d/map",ID_ROBOT);
        msg.header.frame_id = string;
    }

    msg.pose.pose.position.x = xPos[idx]; //send odometry.x
    msg.pose.pose.position.y = yPos[idx]; //send odometry.y

    positions_pub.publish(msg);
    ros::spinOnce();

}

/**
 * James Code to log positions
 *
 */
void PatrolAgent::receive_positions()
{
    // Log all robot positions every second

    if (ID_ROBOT == 0) {

        double current_time = ros::Time::now().toSec();
        if (current_time - last_pos_log > 0.99) {

            last_pos_log = current_time;

            fprintf(positionstimecsvfile, "%.1f;",current_time);

            for (uint8_t ndx=0; ndx < TEAMSIZE; ndx++) {
                fprintf(positionstimecsvfile, "%.3f; %.3f;",xPos[ndx],yPos[ndx]);
            } 
            fprintf(positionstimecsvfile, "\n");
            fflush(positionstimecsvfile);
        }
    }
}

/**
 * Gets positions from ALL other agents and collates them.
 * Also updates TEAMSIZE here for some stupid reason
 * @param msg
 */
void PatrolAgent::positionsCB(const nav_msgs::Odometry::ConstPtr& msg) { //construir tabelas de posições
        
//     printf("Construir tabela de posicoes (receber posicoes), ID_ROBOT = %d\n",ID_ROBOT);    

    char id[20]; //robot identifier that sends the position msg...
    strcpy( id, msg->header.frame_id.c_str() );
    //int stamp = msg->header.seq;
//     printf("robot q mandou msg = %s\n", id);
    
    // Build Positions Table
    
    if (ID_ROBOT>-1){
    //verify id "XX" of robot: (string: "robot_XX/map")
    
        char str_idx[4];
        uint i;

        //To skip past the "ROBOT_" part of string
        for (i=6; i<10; i++){
            if (id[i]=='/'){
                str_idx[i-6] = '\0';
                break;
            }else{
                str_idx[i-6] = id[i];
            }
        }
        
        int idx = atoi (str_idx);
    //  printf("id robot q mandou msg = %d\n",idx);
        
        if (idx >= TEAMSIZE && TEAMSIZE <= NUM_MAX_ROBOTS){
            //update teamsize:
            TEAMSIZE = idx+1;
        }
        
        if (ID_ROBOT != idx){  //Ignore own positions   
            xPos[idx]=msg->pose.pose.position.x;
            yPos[idx]=msg->pose.pose.position.y;        
        }   
//      printf ("Position Table:\n frame.id = %s\n id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx, xPos[idx], idx, yPos[idx] );       
    }

    receive_positions();
}


void PatrolAgent::send_results() { 

}

// simulates blocking send operation with delay in communication
void PatrolAgent::do_send_message(std_msgs::Int16MultiArray &msg) {
	if (communication_delay>0.001) {
    	//double current_time = ros::Time::now().toSec();
    	//if (current_time-last_communication_delay_time>1.0) { 
	        //ROS_INFO("Communication delay %.1f",communication_delay);
	        ros::Duration delay(communication_delay); // seconds
	        delay.sleep();
	        //last_communication_delay_time = current_time;
        //}
    }    
    results_pub.publish(msg);
    ros::spinOnce();
}


void PatrolAgent::receive_results() {

}

void PatrolAgent::send_interference(){
    //interference: [ID,msg_type]
    
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    printf("Send Interference: Robot %d\n",value);   
    
    std_msgs::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(INTERFERENCE_MSG_TYPE);
    
    results_pub.publish(msg);   
    ros::spinOnce();
}




void PatrolAgent::resultsCB(const std_msgs::Int16MultiArray::ConstPtr& msg) { 
    
    std::vector<signed short>::const_iterator it = msg->data.begin();    
    
    vresults.clear();
    
    for (size_t k=0; k<msg->data.size(); k++) {
        vresults.push_back(*it); it++;
    } 

    int id_sender = vresults[0];
    int msg_type = vresults[1];
    
    //printf(" MESSAGE FROM %d TYPE %d ...\n",id_sender, msg_type);
    
    // messages coming from the monitor
    if (id_sender==-1 && msg_type==INITIALIZE_MSG_TYPE) {
        if (initialize==true && vresults[2]==100) {   //"-1,msg_type,100,seq_flag" (BEGINNING)
            ROS_INFO("Let's Patrol!\n");
            double r = 1.0 * ((rand() % 1000)/1000.0);

            //TODO if sequential start
            //r = DELTA_TIME_SEQUENTIAL_START * ID_ROBOT;

            ros::Duration wait(r); // seconds

            printf("Wait %.1f seconds (init pos:%s)\n",r,initial_positions.c_str());

            wait.sleep();
            initialize = false;
        }

#if SIMULATE_FOREVER == false
        if (initialize==false && vresults[2]==999) {   //"-1,msg_type,999" (END)
            ROS_INFO("The simulation is over. Let's leave");
            end_simulation = true;     
        }
#endif        
    }
    
    if (!initialize) {
#if 0
        // communication delay
        if(ID_ROBOT>-1){
            if ((communication_delay>0.001) && (id_sender!=ID_ROBOT)) {
                    double current_time = ros::Time::now().toSec();
                    if (current_time-last_communication_delay_time>1.0) { 
                            ROS_INFO("Communication delay %.1f",communication_delay);
                            ros::Duration delay(communication_delay); // seconds
                            delay.sleep();
                            last_communication_delay_time = current_time;
                }
            }
            bool lost_message = false;
            if ((lost_message_rate>0.0001)&& (id_sender!=ID_ROBOT)) {
                double r = (rand() % 1000)/1000.0;
                lost_message = r < lost_message_rate;
            }
            if (lost_message) {
                ROS_INFO("Lost message");
            }
        }
#endif
            receive_results();
    }

    ros::spinOnce();
  
}

/**
 * Check if robots are near each other, and communicate belief vector
 * @param robot_id Robot ID that started the comparison check
 * @return Bool if comparison occurred or not
 */

bool PatrolAgent::check_comparison_range(){
    bool comparison_occurred = false;
    double time_delta, agent_distance, now;
    std::vector<float> second_agent_belief;

    //ROS_INFO("howdy there partner");
        //cant be this causing crash
    for(int second_robot=0; second_robot<TEAMSIZE; second_robot++){
        if (second_robot != ID_ROBOT){

            agent_distance = sqrt(pow( (xPos[ID_ROBOT] - xPos[second_robot]), 2) + pow((yPos[ID_ROBOT] - yPos[second_robot]), 2));

            if(agent_distance <= communication_distance){
                now = ros::Time::now().toSec();
                //TODO how up update communication history on second_robot?
                time_delta = now - communication_history[second_robot];
                if(time_delta >= communication_period){
                    ROS_INFO("I'm going to communicate!");
                    //subscribes to second_agent topic, sets received_second_belief to be false
                    //wait for message to be received
                    PatrolAgent::second_belief_get(second_robot);
                    while(!received_second_belief){
                        //PatrolAgent::second_belief_get(second_robot);

                        ros::Rate loop_rate(2); //2 sec
                        loop_rate.sleep();
                        ros::spinOnce();

                        ROS_INFO("Waiting for belief message to be received");

                    }
                    three_val_pairwise_comp(belief_states, second_agent_belief_states);

                    ROS_INFO("Beliefs were exchanged and initiated by agents %d and %d", ID_ROBOT, second_robot);
                    now = ros::Time::now().toSec();
                    communication_history[second_robot] = now;
                    pub_beliefs(belief_states, ID_ROBOT);
                    pub_beliefs(belief_states, second_robot);
                    comparison_occurred = true;
                }
            }
        }
    }
    return comparison_occurred;
}

/**
 * Callback function to take data from topic and copy it into second_agent_belief
 * @param msg Callback msg
 */
void PatrolAgent::second_belief_get(int second_robot_ID){
    char topic_name[40];
    sprintf(topic_name, "robot_%d/belief_state", second_robot_ID);
    ros::NodeHandle nh;

    received_second_belief = false;     //set receive flag to be false before subscribing
    second_belief_sub.shutdown();       //shut down previous subscriber before starting new one
    second_belief_sub = nh.subscribe<std_msgs::Float32MultiArray>(topic_name, 1, boost::bind(&PatrolAgent::second_belief_CB, this, _1));
    ros::spinOnce();
}

void PatrolAgent::second_belief_CB(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i=0; i < dimension ; i++){
        second_agent_belief_states = msg->data;
    }
    received_second_belief = true;
}

/**
 * Callback function for agent's OWN belief topic
 * Takes data from topic and updates local belief_states
 * @param msg Reference to topic data
 */
void PatrolAgent::personal_belief_topic_CB(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i=0; i < dimension ; i++){
        belief_states[i] = msg->data[i];
    }
}

/**
 * Function to publish the beliefs to topics based on robot_id passed as argument and data of belief_states
 * @param belief_states Data of belief states as vector<float>
 * @param robot_id ID of agent to publish to
 */
void PatrolAgent::pub_beliefs(std::vector<float>& belief_states, int robot_id){
    std_msgs::Float32MultiArray msg;

    //creates array dimension object
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = dimension;
    msg.layout.dim[0].stride = 1;

    //creates string with robot_id in label to retrieve by monitor
    msg.layout.dim[0].label = std::to_string(robot_id);
    msg.data.clear();
    //inserts data from belief_states
    msg.data.insert(msg.data.end(), belief_states.begin(), belief_states.end());

    belief_publisher[robot_id].publish(msg);

    ros::spinOnce();
}
/**
 * Performs pairwise comparison using three values: true, false and don't know
 * @param first_agent_belief Reference to vector of first agent belief
 * @param second_agent_belief Reference to vector of second agent belief
 * @param dimension Number of nodes in graph
 */
void PatrolAgent::three_val_pairwise_comp(std::vector<float>& first_agent_belief, std::vector<float>& second_agent_belief){

    double yes = 1.0;
    double no = 0.0;

    for(int i=0; i < dimension; i++){
        if(first_agent_belief[i] != second_agent_belief[i]){

            if( ( first_agent_belief[i] != 0.5 ) && ( second_agent_belief[i] != 0.5 ) ){
                first_agent_belief[i] = 0.5;
            }
            if( (first_agent_belief[i] == yes) || (second_agent_belief[i] == yes)){
                first_agent_belief[i] = yes;
            }
            else{
                first_agent_belief[i] = no;
            }
        }
    }
    second_agent_belief = first_agent_belief;
}
/**
 * Calls anomaly service, checks if anomaly is present there, prints information
 * Publishes anomaly status to belief topic
 * @param vertex_arrived The vertex agent arrived at to check if there is an anomaly there
 */
void PatrolAgent::call_anomaly_service(int vertex_arrived){
    patrolling_sim::anomaly_service srv;
    std::string anomaly = "no";


    srv.request.robot_ID = ID_ROBOT;
    srv.request.graph_node_ID = vertex_arrived;
    if(anomaly_client.call(srv)){
        if(srv.response.anomaly_status){
            anomaly = "an";
        }
        ROS_INFO("I saw %s anomaly", anomaly.c_str());
        belief_states[vertex_arrived] = srv.response.anomaly_status;

        pub_beliefs(belief_states, ID_ROBOT);
    }
    else{
        ROS_ERROR("Failed to call service anomaly_node");
    }
}

void scenario_name(char* name, const char* graph_file, const char* teamsize_str)
{
    uint i, start_char=0, end_char = strlen(graph_file)-1;

    for (i=0; i<strlen(graph_file); i++){
        if(graph_file[i]=='/' && i < strlen(graph_file)-1){
            start_char = i+1;
        }

        if(graph_file[i]=='.' && i>0){
            end_char = i-1;
            break;
        }
    }

    for (i=start_char; i<=end_char; i++){
        name [i-start_char] = graph_file [i];
        if (i==end_char){
            name[i-start_char+1] = '\0';
        }
    }

    strcat(name,"_");
    strcat(name,teamsize_str);
}