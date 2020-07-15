#ifndef GENERATE_TRAJECTORY_H
#define GENERATE_TRAJECTORY_H

#include "spline.h"
#include "vehicle_database.h"

using std::cout;
using std::endl;

int TargetLane(int lane,int state){
    if(state==1){lane-=1;}
    if(state==2){lane+=1;}
    return lane;
}

double TargetS(Vehicle& ego_vehicle,int state,VehicleDatabase& vehicle_database,vector<double>& map_waypoints_x,vector<double>& map_waypoints_y,vector<double>& map_waypoints_s){
  double front_vehicle_s; //Get front vehicle s from sensor_fusion
  double front_vehicle_speed; //Get front vehicle speed from sensor_fusion

  int index=ClosestWaypoint(ego_vehicle.x,ego_vehicle.y,map_waypoints_x,map_waypoints_y);
  double s_value=map_waypoints_s[index];

  double x_wp=map_waypoints_x[index];
  double y_wp=map_waypoints_y[index];

  

  if((s_value<ego_vehicle.s and ego_vehicle.s<s_last_waypoint) or (distance(ego_vehicle.x,ego_vehicle.y,x_wp,y_wp)<waypoint_distance_threshold and ego_vehicle.s>=s_last_waypoint) ){
    index+=1;
    index=index%map_waypoints_x.size();
    s_value=map_waypoints_s[index];
  }
  
    index=vehicle_database.VehicleInFront(ego_vehicle);
    Vehicle front_vehicle=vehicle_database.vehicles[index];

    if(state==3 and front_vehicle.speed<speed_limit){
      double time_to_collision=vehicle_database.DistanceBetweenVehicles(ego_vehicle,front_vehicle)/(front_vehicle_speed+ego_vehicle.speed)*2;
      s_value=front_vehicle_s+0.9*time_to_collision*front_vehicle_speed-stopping_distance;
    }

return s_value;
}

void append_setpoint(vector<double>& s_setpoints,vector<double>& d_setpoints,vector<double>& time_setpoints,vector<double>& values){
  s_setpoints.push_back(values[0]);
  d_setpoints.push_back(values[1]);
  time_setpoints.push_back(values[3]);
}

vector<vector<double>> GenerateTrajectory(Vehicle& ego_vehicle,int state,VehicleDatabase& vehicle_database,vector<double>& map_waypoints_x,vector<double>& map_waypoints_y,vector<double>& map_waypoints_s,nlohmann::basic_json<>& previous_path_x,nlohmann::basic_json<>& previous_path_y,bool use_time=true){
    
    int target_lane=TargetLane(ego_vehicle.lane,state);
    double d_end=(target_lane +0.5)*lane_width;
    //cout<<"Target d="<<d_end<<endl;

    double s_end=ego_vehicle.s+30;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    //Get front vehicle speed to calculate the end time.
    double target_speed=speed_limit;

    if(state==3){
        //If the state is 3, there's definitely a vehicle there
      int index=vehicle_database.VehicleInFront(ego_vehicle);
      target_speed=vehicle_database.vehicles[index].speed;
    }

    double end_time=abs(target_speed-ego_vehicle.speed)/(0.8*max_accn);

    tk::spline s_spline,d_spline;

    vector<double> s_setpoints,d_setpoints,time_setpoints;

    time_setpoints={-timestep_duration};

    //cout<<"Next x="<<previous_path_x[0]<<endl;

    if(previous_path_x[0]==nlohmann::basic_json<>::value_t::null or previous_path_x.size()<2){

    	  ego_vehicle.speed=std::max(ego_vehicle.speed,min_speed);
        s_setpoints.push_back(ego_vehicle.s+1*timestep_duration*ego_vehicle.speed);
        d_setpoints.push_back(ego_vehicle.d); 
        time_setpoints.push_back(time_setpoints.back()+timestep_duration);    

        //cout<<"Visit"<<s_setpoints.back()<<"At"<<time_setpoints.back()<<endl;

        s_setpoints.push_back(ego_vehicle.s+2*timestep_duration*ego_vehicle.speed);
        d_setpoints.push_back(ego_vehicle.d);
        time_setpoints.push_back(time_setpoints.back()+timestep_duration);
        //cout<<"Visit"<<s_setpoints.back()<<"At"<<time_setpoints.back()<<endl;

      }

    else{
      int n_points=0;
      cout<<"Previous Path Available"<<endl;

      for(int i=0;i<=previous_path_x.size()-1 and n_points<15;i++){  
        vector<double> frenets=getFrenet(previous_path_x[i],previous_path_y[i],ego_vehicle.yaw,map_waypoints_x,map_waypoints_y);

      	s_setpoints.push_back(frenets[0]);
        d_setpoints.push_back(frenets[1]);
        time_setpoints.push_back(time_setpoints.back()+timestep_duration);
        n_points++;
		    
        //We won't ever be at the edge of a trajectory as we issue a new path much before that    
      }
      
    }
    
    time_setpoints.erase(time_setpoints.begin());

    s_setpoints.push_back(s_end);
    d_setpoints.push_back(d_end);
    time_setpoints.push_back(end_time);

    s_setpoints.push_back(s_end+target_speed*timestep_duration); //To obey the velocity constraint
    d_setpoints.push_back(d_end);
    time_setpoints.push_back(end_time+timestep_duration);

    s_setpoints.push_back(s_end+30); //To have a smooth trajectory at the waypoint, we calculate it till one waypoint ahead
    d_setpoints.push_back(d_end);
    time_setpoints.push_back(end_time+30/target_speed);

    s_setpoints.push_back(s_end+30+target_speed*timestep_duration); //To obey the velocity constraint
    d_setpoints.push_back(d_end);
    time_setpoints.push_back(end_time+timestep_duration+30/target_speed);

    /*
    s_setpoints.push_back(s_end+60); //To have a smooth trajectory at the waypoint, we calculate it till two waypoint ahead
    d_setpoints.push_back(d_end);
    time_setpoints.push_back(end_time+60/target_speed);

    s_setpoints.push_back(s_end+60+target_speed*timestep_duration); //To obey the velocity constraint
    d_setpoints.push_back(d_end);
    time_setpoints.push_back(end_time+timestep_duration+60/target_speed);
	*/

    //First generate s and d as functions of time. 
    s_spline.set_points(time_setpoints,s_setpoints,true);
    d_spline.set_points(time_setpoints,d_setpoints,true);
    double t=0;

    if(use_time){
      while(t<end_time+0/target_speed){

      double s=s_spline(t);
      if(s>s_max){s-=s_max;}

      //cout<<"s="<<s<<"At t="<<t<<endl;

      double d=d_spline(t);
      vector<double> cartesian_coords=getXY(s,d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
      
      next_x_vals.push_back(cartesian_coords[0]);
      next_y_vals.push_back(cartesian_coords[1]);
      t+=timestep_duration;
    }

    vector<vector<double>> trajectory={next_x_vals,next_y_vals};
    return trajectory;  
    }
    
    else{

      tk::spline ds_spline;
      ds_spline.set_points(s_setpoints,d_setpoints,true);

      double vel=ego_vehicle.speed;
      double current_s=ego_vehicle.s;

      while(current_s<s_end){

        double ds=vel*timestep_duration;
        current_s+=ds;

        double current_d=ds_spline(current_s);
        vector<double> cartesian_coords=getXY(current_s,current_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
        
        next_x_vals.push_back(cartesian_coords[0]);
        next_y_vals.push_back(cartesian_coords[1]);

        if(vel<speed_limit-3){vel+=0.9*max_accn*timestep_duration;}        

      }

      return {next_x_vals,next_y_vals};
    }

    //Now, correct s to be within the bounds, and calculate x and y and add them to the trajectory.


}
#endif