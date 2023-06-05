#include <sstream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <cstdlib>

#include "patrolling_sim/anomaly_service.h"

#define RANDOM  0
#define DETERMINED  1
#define EVEN  2
#define ODD  3
#define FIRST_LINE  12

using namespace std;

vector<int> anomaly_time_vector;
int number_anomalies;
int dimension,time_now;
vector<bool> anomaly_presence_vector;
int anomaly_assignment_type, communication_distance, communication_period;
string anomaly_param_name;

const std::string PS_path = ros::package::getPath("patrolling_sim");
string local_anomaly_dir = "/params/anomaly/anomaly_params.txt";

bool anomaly_presence(patrolling_sim::anomaly_service::Request  &req, patrolling_sim::anomaly_service::Response &res){

    for(int i=0; i < (anomaly_time_vector.size() + 1); i++){
        time_now = ros::Time::now().toSec();

        if ( time_now >= anomaly_time_vector[i]){
            res.anomaly_status = anomaly_presence_vector[req.graph_node_ID];
        }
    }


   return true;
}

void anomaly_node_init( int dimension ) {
     int random_node = 0;
     bool anomaly_written = false;
    srand(time(NULL));

    //create anomaly_presence_vector size from dimension
     anomaly_presence_vector.resize(dimension);

     //fill anomaly_presence_vector with false
     std::fill(anomaly_presence_vector.begin(), anomaly_presence_vector.end(), false);

     //location and filename
     anomaly_param_name = PS_path + local_anomaly_dir;
     ROS_INFO("anomaly location is %s", anomaly_param_name.c_str());
     FILE *file;
     file = fopen(anomaly_param_name.c_str(), "r");

     if (file == NULL) {
         ROS_INFO("Cannot open filename %s", anomaly_param_name.c_str());
         ROS_BREAK();
     } else {
         ROS_INFO("Anomaly file opened; retrieving anomaly parameters.\n");

         uint j;
         char temp;
         int r;

         //'Gets rid' of first few comment lines
         for(int i=0; i < (FIRST_LINE - 1) ; i++) {
             char buffer[100];
             fgets(buffer, 100, file);
         }

         r = fscanf(file, "%d", &number_anomalies);
         ROS_INFO("number_anomalies is - value %d", number_anomalies);

         if(number_anomalies == 0){
             ROS_INFO("Zero anomalies defined, anomaly node not enabled - value %d", number_anomalies);
             ROS_BREAK();
         }

         //Define the size of anomaly time vector from param file
         anomaly_time_vector.resize(number_anomalies);

         r = fscanf(file, "%d", &anomaly_assignment_type);
         ROS_INFO("anomaly_assignment_type is - value %d", anomaly_assignment_type);
         r = fscanf(file, "%d", &communication_distance);
         ROS_INFO("communication_distance is - value %d", communication_distance);

         r = fscanf(file, "%d", &communication_period);
         ROS_INFO("communication_period is - value %d", communication_period);


         //for the number of anomalies, take a time value from file
         int value_from_file;
         for(int i=0; i < number_anomalies; i++){
             r=fscanf(file, "%d", &anomaly_time_vector[i]);
         }


         //TODO error handling for param file not being long enough
         if (anomaly_assignment_type == DETERMINED) {
             int placement_from_file = 0;
             for (int k = 0; k < number_anomalies; k++) {
                 //get the placement of anomalies on the graph
                 r = fscanf(file, "%d", &placement_from_file);
                 if( (placement_from_file + 1 ) > dimension){
                     ROS_INFO("Error in param file, anomaly out of dimension");
                     ROS_BREAK();
                 }
                 anomaly_presence_vector[placement_from_file] = true;
                 ROS_INFO("Anomaly placed at node %d",placement_from_file);
             }
         }

     if (anomaly_assignment_type == RANDOM) {
         for (int i = 0; i < number_anomalies; i++) {
             anomaly_written = false;
             while (!anomaly_written) {
                 //generates node ID between 0 - dimension
                 random_node = (int)(rand() % dimension);
                 //if there ISN'T an anomaly already there, set one to be there and break out of while loop
                 if (anomaly_presence_vector[random_node] == false ) {
                     anomaly_presence_vector[random_node] = true;
                     anomaly_written = true;
                     ROS_INFO("An anomaly has been generated at node %d", random_node);
                 }
             }

         }
     }

     fclose(file);

        if (anomaly_assignment_type == EVEN) {
             //TODO create even assignment
        }
        if (anomaly_assignment_type == ODD) {
             //TODO create odd assignment
        }
     }
     patrolling_sim::anomaly_service service_object;
     ros::service::call("/anomaly_service", service_object);
}


void get_map_dimension(){
     //get the map dimension from map file
     //set dimension to this value
 }

 int main(int argc, char **argv) {

     ros::init(argc, argv, "anomaly_node");
     ros::NodeHandle n;
     ros::Rate loop_rate(0.5);
     n.setParam("/anomaly_distance", communication_distance);
     n.setParam("/anomaly_comm_period", communication_period);

     //TODO call anomaly node from start_experiment.py


     if(!( n.getParam("/graph_dimension", dimension))) {
         ROS_ERROR("Dimension parameter unable to be retrieved, anomaly node exiting.");
         ROS_BREAK();
     }


     anomaly_node_init(dimension);

     ros::ServiceServer service = n.advertiseService("/anomaly_service", anomaly_presence);
     //TODO way to communicate what time anomaly was generated at
     //ROS_INFO("Ready to activate an anomaly at time" , anomaly_time);

     while(ros::ok()){
         loop_rate.sleep();
         ROS_INFO("anomaly node running");
         ros::spinOnce();
     }
    return 0;

}
