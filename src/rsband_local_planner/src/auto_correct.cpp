#include"rsband_local_planner/auto_correct.h"


namespace rsband_local_planner{

point_list::point_list()
{
	pi=3.1415926;
	gain_angle = 1;
	unit_distance = 100;
	warning_distance = 2; 
	limit_distance = 0.2;
	angle_max = 1.57;
	angle_min = -1.57; 	
}



void point_list::output()
{
	if(warning_point.empty())
		printf("null\n");
	else
	{
		int count = warning_point.size();
    	for (int i = 0;i<count;i++)
    	{
			ROS_INFO("in dis=%.2f,ang=%.2f",warning_point[i].distance,warning_point[i].angle);
    		//printf("dis:%f\t | ang:%f\n",warning_point[i].distance,warning_point[i].angle);
    	}	
	} 
}


void point_list::append(float dis,float ang)
{
	struct Obs_point in;
	in.distance=dis;
	in.angle=ang;
	warning_point.push_back(in);	
}


void point_list::clearlist()
{
	warning_point.clear();
}


    
void point_list::sortlist()
{
	if(warning_point.empty())
		;
	else
	{
		int count = warning_point.size();
   		for(int i=0;i<count-1;i++)
    	{
        	for(int j=0;j<count-i-1;j++)
        	{
        		if(warning_point[j].distance > warning_point[j+1].distance)
				{
					float temp_d = warning_point[j].distance;
					warning_point[j].distance = warning_point[j+1].distance;
					warning_point[j+1].distance = temp_d;
				
					float temp_a = warning_point[j].angle;
					warning_point[j].angle = warning_point[j+1].angle;
					warning_point[j+1].angle = temp_a;
				}
        	}
    	}
	} 
}




void point_list::v_vector(float add)
{
 
	if(warning_point[0].distance < warning_distance && warning_point[0].distance > limit_distance)
	{
		if(warning_point[0].angle > 0)
			out_point.angle = warning_point[0].angle - add;
		else if(warning_point[0].angle <= 0)
			out_point.angle = warning_point[0].angle + add;
			
		if(out_point.angle > angle_max)
			out_point.angle = angle_max;
		else if(out_point.angle <= angle_min)
			out_point.angle = angle_min;
			
		out_point.distance =  unit_distance;
	} 
	

	else if(warning_point[0].distance <= limit_distance && warning_point[0].distance >= 0)
	{
		if(warning_point[0].angle > 0)
			out_point.angle = angle_min;
		else if(warning_point[0].angle <= 0)
			out_point.angle = angle_max;
	
		out_point.distance = unit_distance;		
	}
}


}//end namespace