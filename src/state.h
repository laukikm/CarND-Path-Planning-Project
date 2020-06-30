#ifndef STATE_H
#define STATE_H

int State(nlohmann::basic_json<>& sensor_fusion, Vehicle& ego_vehicle,vector<double> map_waypoints_x,vector<double> map_waypoints_y){

	/*
	0: Track Speed
	1: Turn Left
	2: Turn Right
	3: Follow Leader
  4: Stop instantly
	*/

  bool vehicle_in_front=false;
	bool vehicle_to_right=false;
	bool vehicle_to_left=false;
	bool stop_instantly=false;

	for(auto iter=sensor_fusion.begin();iter!=sensor_fusion.end();iter++){
		vector<double> car_data=*iter;

		double x=car_data[1];
		double y=car_data[2];

		double s=car_data[5];
		double d=car_data[6];

		double vx=car_data[3];
		double vy=car_data[4];
		double speed=sqrt(vx*vx+vy*vy);

		if(distance(x,y,ego_vehicle.x,ego_vehicle.y)<front_distance_tolerance){


			if(abs(d-ego_vehicle.d)<same_lane_tolerance){
        vehicle_in_front=true;
        if(distance(x,y,ego_vehicle.x,ego_vehicle.y)<stopping_distance){stop_instantly=true;}
      }
	

  		else if(d<ego_vehicle.d and speed<speed_limit){vehicle_to_left=true;}
			else if(d>ego_vehicle.d and speed<speed_limit){vehicle_to_right=true;}

			//cout<<"Other Vehicle d="<<d<<endl;
			//cout<<"Ego Vehicle d="<<ego_vehicle.d<<endl;
		}

		if(distance(x,y,ego_vehicle.x,ego_vehicle.y)<back_distance_tolerance){

			if(d<ego_vehicle.d-same_lane_tolerance){vehicle_to_left=true;}
			else if(d>ego_vehicle.d+same_lane_tolerance){vehicle_to_right=true;};
		}
	}

  if(stop_instantly){return 4;}
	if(!vehicle_in_front){return 0;}
	
	//cout<<"vehicle_to_left="<<vehicle_to_left<<endl;
	//cout<<"vehicle_to_right="<<vehicle_to_right<<endl;

	if(vehicle_to_left and vehicle_to_right){return 3;}

	else if(!vehicle_to_left){return 1;}

	else if(!vehicle_to_right){return 2;}

	return 0;

}

#endif