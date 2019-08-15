
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
    	
		double pi;
		double gain_angle;
		double unit_distance;
		double warning_distance;
		double limit_distance; 
		double angle_max;
		double angle_min; 	
		
	private:
};
}//end namespace