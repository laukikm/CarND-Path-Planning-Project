#ifndef VEHICLE_DATABASE_H
#define VEHICLE_DATABASE_H


#include "json.hpp"
#include "math.h"
#include "vehicle.h"

class VehicleDatabase{
public:
  vector<Vehicle> vehicles;
  Vehicle ego_vehicle;

  VehicleDatabase(nlohmann::basic_json<>& sensor_fusion){

  	for(auto iter=sensor_fusion.begin();iter!=sensor_fusion.end();iter++){
  		vector<double> car_data=*iter;

	    double x=car_data[1];
	    double y=car_data[2];

	    double s=car_data[5];
	    double d=car_data[6];

	    double vx=car_data[3];
	    double vy=car_data[4];
	    double speed=sqrt(vx*vx+vy*vy);

	    double yaw=atan2(vy,vx);

	    Vehicle current_vehicle=Vehicle(x,y,s,d,yaw,speed);
	    vehicles.push_back(current_vehicle);
  	}
  }

  double DistanceBetweenVehicles(Vehicle& v1, Vehicle& v2){
    return distance(v1.x,v1.y,v2.x,v2.y);
  }

  bool IsVehicleAhead(Vehicle& v1, Vehicle& v2){
    //Is v2 ahead of v1. This function is used only when v1 and v2 are close
    bool is_v2_ahead=false;
    if(v1.s<v2.s and v1.s<s_max-front_distance_tolerance){
      is_v2_ahead=true;
    }

    if(v1.s>v2.s and v1.s>s_max-front_distance_tolerance){
      is_v2_ahead=true;
    }
    return is_v2_ahead;
  }

  bool SameLane(Vehicle& v1, Vehicle& v2){
  	return abs(v1.d-v2.d)<same_lane_tolerance;
  }

  bool VehicleToLeft(Vehicle ego_vehicle){ //Provides a list of vehicles to the left.
    int is_left_lane_occupied=false;
    for(auto iter=vehicles.begin();iter<vehicles.end();iter++){
      Vehicle current_vehicle=*iter;

      if(DistanceBetweenVehicles(ego_vehicle,current_vehicle)<front_distance_tolerance and IsVehicleAhead(ego_vehicle,current_vehicle)){
      	if(current_vehicle.d<ego_vehicle.d-same_lane_tolerance){is_left_lane_occupied=true;}
      }

      if(DistanceBetweenVehicles(ego_vehicle,current_vehicle)<back_distance_tolerance and !IsVehicleAhead(ego_vehicle,current_vehicle)){
      	if(current_vehicle.d<ego_vehicle.d-same_lane_tolerance){is_left_lane_occupied=true;}
      }
    }
    return is_left_lane_occupied;
  }

	bool VehicleToRight(Vehicle ego_vehicle){ //Provides a list of vehicles to the left.
	int is_right_lane_occupied=false;
	for(auto iter=vehicles.begin();iter<vehicles.end();iter++){
	  Vehicle current_vehicle=*iter;

	  if(DistanceBetweenVehicles(ego_vehicle,current_vehicle)<front_distance_tolerance and IsVehicleAhead(ego_vehicle,current_vehicle)){
	  	if(current_vehicle.d>ego_vehicle.d+same_lane_tolerance){is_right_lane_occupied=true;}
	  }

	  if(DistanceBetweenVehicles(ego_vehicle,current_vehicle)<back_distance_tolerance and !IsVehicleAhead(ego_vehicle,current_vehicle)){
	  	if(current_vehicle.d>ego_vehicle.d+same_lane_tolerance){is_right_lane_occupied=true;}
	  }
	}
	return is_right_lane_occupied;
	}  

	int VehicleInFront(Vehicle& ego_vehicle){
		int index=-1;
		for(auto iter=vehicles.begin();iter<vehicles.end();iter++){
		  Vehicle current_vehicle=*iter;
		  index++;
		  if(DistanceBetweenVehicles(ego_vehicle,current_vehicle)<front_distance_tolerance and IsVehicleAhead(ego_vehicle,current_vehicle)){
		  	if(SameLane(ego_vehicle,current_vehicle)){return index;}
		  }

		}
		return index;
		}

	double ClosestDistance(Vehicle& ego_vehicle){
		double min_distance=10000;
		for(auto iter=vehicles.begin();iter<vehicles.end();iter++){
		  Vehicle current_vehicle=*iter;
		  min_distance=std::min(min_distance,DistanceBetweenVehicles(ego_vehicle,current_vehicle));
	}
	}		

	int State(Vehicle& ego_vehicle){
		/*
		0: Track Speed
		1: Turn Left
		2: Turn Right
		3: Stop Instantly
		*/

		bool vehicle_in_front=VehicleToLeft(ego_vehicle);
		bool vehicle_to_right=VehicleToRight(ego_vehicle);
		bool vehicle_to_left=VehicleInFront(ego_vehicle)==-1;
		
		bool stop_instantly=ClosestDistance(ego_vehicle)<stopping_distance;

		if(stop_instantly){return 3;}
		if(!vehicle_in_front){return 0;}

		if(!vehicle_to_left and ego_vehicle.lane>0){
			return 1;
		}

		return 2;
	}

};

#endif