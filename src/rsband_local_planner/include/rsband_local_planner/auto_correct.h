
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
		void v_vector();
    	struct Obs_point out_point;
    	vector<struct Obs_point> warning_point;
    	
		double pi;
		double gain_angle;
		double unit_distance;
		double warning_distance;
		double limit_distance; 
		double angle_max;
		double angle_min; 
		int signal_front=0;
		int signal_left=0;
		int signal_right=0;
		int signal_back=0;
		int left_count =0;
    int right_count=0;
	int front_count=0;
	int back_count=0;
		
	private:
};
}//end namespace