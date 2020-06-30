#ifndef VEHICLE_H
#define VEHICLE_H

#include "globals.h"
#include "helpers.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

class Vehicle{
public:
	double x,y,s,d,yaw,speed;

  int lane; //0=left,1=center,2=right
	int id;

  double vx,vy;
	Vehicle(double x=0,double y=0,double s=0,double d=0,double yaw=0,double speed=0){
		this->x=x;
		this->y=y;
		this->s=s;
		this->d=d;
		this->yaw=yaw;
    this->lane=int(d/lane_width);
    this->speed=speed;

    this->vx=speed*cos(yaw);
    this->vy=speed*sin(yaw);


	}

};

#endif