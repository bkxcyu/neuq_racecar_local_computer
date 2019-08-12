
#include <stdio.h>
#include <malloc.h>
#include <iostream>
using namespace std; 
#define POINTLEN sizeof(struct Obs_point)
#define LINK_HEAD_D 99999
#define LINK_HEAD_A 99999
namespace rsband_local_planner{
struct Obs_point
{
	float distance;
	float angle;
	struct Obs_point *next;
};

struct Out_point
{
	float dis;
	float ang;	
};

class point_list
{
	public:
		point_list();
		struct Obs_point *creatlist();//�����ǿ����� 
		int append(struct Obs_point *head,float dis,float ang);//��������������input 
		void output(struct Obs_point *head);//������� 
		struct Obs_point *clearlist(struct Obs_point *head);//�������������ͷ 
		struct Obs_point *sortlist(struct Obs_point *head);//�������� 
		struct Obs_point *getlastnode(struct Obs_point *head);//�õ����������һ�����
		void v_vector(struct Obs_point *head);
		//���б��� 
		struct Obs_point *warning_point;
    	struct Obs_point *last_point;
    	struct Out_point out_point;//������� 
	
		float pi;
		int gain_angle;//�Ƕ����� 
		int unit_distance;//��λ���� 
		int warning_distance;//Ԥ������ 
		int limit_distance;//���޾��� 
		int angle_max;
		int angle_min; 	

	private:

};
}//end namespace