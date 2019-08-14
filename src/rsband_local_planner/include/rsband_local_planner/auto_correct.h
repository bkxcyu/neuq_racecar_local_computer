
#include <stdio.h>
//#include "iostream"
#include "vector"
#include <ros/ros.h>

using namespace std;

namespace rsband_local_planner{

struct Obs_point
{
    float distance;
    float angle;
};

class point_list
{
	public:
		point_list();		
		void append(float dis,float ang);
		void output();
		void clearlist();
		void sortlist(); 	
		void v_vector(float add);
    	struct Obs_point out_point;
    	vector<struct Obs_point> warning_point;
    	
		float pi;
		float gain_angle;
		float unit_distance;
		float warning_distance;
		float limit_distance; 
		float angle_max;
		float angle_min; 	
		
	private:
};
}//end namespace