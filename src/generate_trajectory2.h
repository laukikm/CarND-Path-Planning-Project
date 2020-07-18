#ifndef GENERATE_TRAJECTORY2_H
#define GENERATE_TRAJECTORY2_H

#include "spline.h"
#include "vehicle_database.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using std::cout;
using std::endl;

int TargetLane(int lane,int state){
    if(state==1){lane-=1;}
    if(state==2){lane+=1;}
    return lane;
}

void append_setpoint(vector<double>& s_setpoints,vector<double>& d_setpoints,vector<double>& time_setpoints,vector<double>& values){
  s_setpoints.push_back(values[0]);
  d_setpoints.push_back(values[1]);
  time_setpoints.push_back(values[3]);
}

vector<double> convert_to_car_frame(double x,double y,Vehicle& ego_vehicle){

	double dx=x-ego_vehicle.x;
	double dy=y-ego_vehicle.y;

	double yaw=deg2rad(ego_vehicle.yaw);

	double car_x=dx*cos(yaw)+dy*sin(yaw);
	double car_y=-dx*sin(yaw)+dy*cos(yaw);

	return {car_x,car_y};
}

vector<double> convert_to_map_frame(double x,double y,Vehicle& ego_vehicle){

	double yaw=deg2rad(ego_vehicle.yaw);

	double map_x=x*cos(yaw)-y*sin(yaw);
	double map_y=x*sin(yaw)+y*cos(yaw);

	map_x+=ego_vehicle.x;
	map_y+=ego_vehicle.y;

	return {map_x,map_y};
}

void print_vector(std::vector<double>& v){
  cout<<"Values=";
  for(auto iter=v.begin();iter!=v.end();++iter){
    cout<<*iter<<" ";
  }
  cout<<endl;
}

vector<double> latency_compensation(double x,double y,Vehicle& ego_vehicle){

    double vel=ego_vehicle.speed*0.447;
    double yaw=deg2rad(ego_vehicle.yaw);

    vector<double> offset={0,0};

    if(distance(x,y,ego_vehicle.x,ego_vehicle.y)<(vel-speed_tolerance)*timestep_duration){
      double prev_x=ego_vehicle.x-vel*cos(yaw)*timestep_duration;
      double prev_y=ego_vehicle.y-vel*sin(yaw)*timestep_duration;

      double offset_x=double(x)-prev_x;
      double offset_y=double(y)-prev_y;

      offset={offset_x,offset_y};  
    }
    return offset;
}

vector<vector<double>> GenerateTrajectory2(Vehicle& ego_vehicle,int state,VehicleDatabase& vehicle_database,vector<double>& map_waypoints_x,vector<double>& map_waypoints_y,vector<double>& map_waypoints_s,nlohmann::basic_json<>& previous_path_x,nlohmann::basic_json<>& previous_path_y){
    
    int target_lane=TargetLane(ego_vehicle.lane,state);
    double d_end=(target_lane +0.5)*lane_width;
    //cout<<"Target d="<<d_end<<endl;

    double s_end=ego_vehicle.s+30;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    //Get front vehicle speed to calculate the end time.
    double target_speed=speed_limit;
    double actual_accn=0.8*max_accn;


    if(state==4){
    	target_speed=0; //Stop instantly
    	actual_accn=max_accn;
    }


    if(state==3){
      
      //If the state is 3, there's definitely a vehicle there
      int index=vehicle_database.VehicleInFront(ego_vehicle);
      target_speed=vehicle_database.vehicles[index].speed;
 
    }

    double end_time=abs(target_speed-ego_vehicle.speed)/actual_accn;

    tk::spline xy_spline;

    vector<double> x_setpoints_relative,y_setpoints_relative;

    //If previous path is unavailable, use current car position for spline generation
    //cout<<(previous_path_x[0]==nlohmann::basic_json<>::value_t::null)<<" This is the first"<<endl;

    vector<double> previous_path_x_vector,previous_path_y_vector;

    if(previous_path_x[0]==nlohmann::basic_json<>::value_t::null){

      //cout<<"No prev points available"<<endl;
    	ego_vehicle.speed=std::max(ego_vehicle.speed,actual_accn*timestep_duration); //We started accelerating

    	double yaw=deg2rad(ego_vehicle.yaw);
    	double x=ego_vehicle.x+ego_vehicle.speed*cos(yaw)*timestep_duration;
    	double y=ego_vehicle.y+ego_vehicle.speed*sin(yaw)*timestep_duration;

    	for(int i=0;i<4;i++){
	    	vector<double> car_coords=convert_to_car_frame(x,y,ego_vehicle);

	    	x_setpoints_relative.push_back(car_coords[0]);
	    	y_setpoints_relative.push_back(car_coords[1]);

	    	x=x+ego_vehicle.speed*cos(ego_vehicle.yaw)*timestep_duration;
	    	y=y+ego_vehicle.speed*sin(ego_vehicle.yaw)*timestep_duration;
    	}
    }
    else{

      int n_points=0;
      //cout<<"Previous Path Available"<<endl;

      for(int i=0;i<=previous_path_x.size()-1 and n_points<10;i++){  
        //vector<double> frenets=getFrenet(previous_path_x[i],previous_path_y[i],ego_vehicle.yaw,map_waypoints_x,map_waypoints_y);

      	vector<double> car_coords=convert_to_car_frame(previous_path_x[i],previous_path_y[i],ego_vehicle);

		    x_setpoints_relative.push_back(car_coords[0]);
    	  y_setpoints_relative.push_back(car_coords[1]);

        previous_path_x_vector.push_back(previous_path_x[i]);
        previous_path_y_vector.push_back(previous_path_y[i]);

      	n_points++;
		//We won't ever be at the edge of a trajectory as we issue a new path much before that    
      } 
    }

    //The points very much ahead 
    for(int i=0;i<1;i++){

    	vector<double> end_points=getXY(s_end+30*i,d_end,map_waypoints_s,map_waypoints_x,map_waypoints_y);

	    double x_end_map=end_points[0];
	    double y_end_map=end_points[1];

	    vector<double> car_coords_ends=convert_to_car_frame(x_end_map,y_end_map,ego_vehicle);

	    double x_end=car_coords_ends[0];
	    double y_end=car_coords_ends[1];

	    x_setpoints_relative.push_back(x_end);
	    y_setpoints_relative.push_back(y_end);
	
    }
    xy_spline.set_points(x_setpoints_relative,y_setpoints_relative,true);

    double vel=ego_vehicle.speed*0.447; //Vehicle speed is in mph, we need m/s
    
    double x=ego_vehicle.x;//+ego_vehicle.speed*cos(ego_vehicle.yaw)*timestep_duration;
   	double y=ego_vehicle.y;

    //cout<<"Is there a problem here?"<<(previous_path_x[0]!=nlohmann::basic_json<>::value_t::null)<<endl;

    vector<double> latency_comp={0,0};

    
    if(ego_vehicle.speed>2){
      latency_comp=latency_compensation(double(previous_path_x[0]),double(previous_path_y[0]),ego_vehicle);
    }
    
    
    if(previous_path_x_vector.size()>0){
    	//cout<<"Previous Path size="<<previous_path_x_vector.size()<<endl;

      for(int i=0;i<std::min(points_from_previous_path,int(previous_path_x_vector.size()-2));i++){
        
        next_x_vals.push_back(previous_path_x_vector[i]+latency_comp[0]);
        next_y_vals.push_back(previous_path_y_vector[i]+latency_comp[1]);

        vel=(double(previous_path_x[i+1])-x)/timestep_duration;

        x=previous_path_x_vector[i+1]+latency_comp[0]; //This is much smaller than the number of points in a path
        y=previous_path_y_vector[i+1]+latency_comp[1];
      }
    }

    vector<double> car_coords=convert_to_car_frame(x,y,ego_vehicle);

    x=car_coords[0];
    y=car_coords[1];

    cout<<"vel= "<<vel<<endl;
    cout<<"Vehicle x="<<ego_vehicle.x<<" first x="<<previous_path_x[0];

   	for(int i=0;i<20;i++){
      double dt=timestep_duration;

      if(vel>target_speed+speed_tolerance){vel-=actual_accn*dt;}
      else if(vel<target_speed-speed_tolerance){vel+=actual_accn*dt;}
   		
      y=xy_spline(x);

      double yaw=deg2rad(ego_vehicle.yaw);
      //cout<< vel <<" ";

   		vector<double> map_coords=convert_to_map_frame(x,y,ego_vehicle);

   		next_x_vals.push_back(map_coords[0]);
   		next_y_vals.push_back(map_coords[1]);

   		x+=vel*dt;

   		//cout<<"x="<<x<<" y="<<y<<endl; 
   	}

    return {next_x_vals,next_y_vals};
}
#endif