#ifndef GLOBALS_H
#define GLOBALS_H

const double front_distance_tolerance=30.0;
const double back_distance_tolerance=20.0;

const double same_lane_tolerance=2.0;

const double speed_limit=22.0; //This is in m/s. Translates to about 50 mph

const double s_max = 6945.554;
const double s_last_waypoint=6914.15;

const double max_accn=5.0;
const double max_jerk=1.0;

const double lane_width=5;

const double stopping_distance=5; //In meters, hopefully;
const double timestep_duration=0.02;

const double waypoint_distance_threshold=5;

const int latency=5; 

#endif