#include"rsband_local_planner/auto_correct.h"



point_list::point_list()
{
	pi=3.1415926;
	gain_angle = 1;//�Ƕ����� 
	unit_distance = 100;//��λ���� 
	warning_distance = 2000;//Ԥ������ 
	limit_distance = 1000;//���޾��� 
	angle_max = pi;
	angle_min = 0; 	
}

// int main()
// {
//     int choice;
// 	bool next;
// 	point_list whosyourdaddy;
    
//     whosyourdaddy.warning_point = whosyourdaddy.creatlist();
    
//     do
//     {
//     	whosyourdaddy.sortlist(whosyourdaddy.warning_point);//���� 
//     	printf("ѡ��1�����  ѡ��2������  ѡ��3�����  ѡ��4�����ĵ�  ѡ��5���������\n - ");
//     	scanf("%d",&choice);
// 		switch(choice)	
// 		{
// 			case 1:
// 				printf("\nOUTPUT:\n"); 
// 				whosyourdaddy.output(whosyourdaddy.warning_point);
// 				break;
// 			case 2:
// 				float dis,ang;
// 				printf("����:\n");
// 				scanf("%f %f",&dis,&ang);
// 				whosyourdaddy.append(whosyourdaddy.warning_point,dis,ang);
// 				break;
// 			case 3:
// 				whosyourdaddy.clearlist(whosyourdaddy.warning_point);
// 				whosyourdaddy.output(whosyourdaddy.warning_point);
// 				break;
// 			case 4:
// 				whosyourdaddy.last_point = whosyourdaddy.getlastnode(whosyourdaddy.warning_point);
// 				whosyourdaddy.output(whosyourdaddy.last_point);
// 				break;
// 			case 5:
// 				whosyourdaddy.last_point = whosyourdaddy.getlastnode(whosyourdaddy.warning_point);
// 				//�ж��Ƿ�Ϊ��ͷ 
// 				if(whosyourdaddy.last_point->distance == LINK_HEAD_D || whosyourdaddy.last_point->distance == LINK_HEAD_A) 
// 					printf("null\n");
// 				else
// 				{
// 					whosyourdaddy.v_vector(whosyourdaddy.last_point);
// 					whosyourdaddy.output(whosyourdaddy.last_point);
// 					printf("the OUT:\n");
// 					printf("dis:%f\t | ang:%f\n",whosyourdaddy.out_point.dis,whosyourdaddy.out_point.ang);
// 				}
// 				break;
// 			default:
//             	printf("OUTPUT:\n"); 
// 				whosyourdaddy.output(whosyourdaddy.warning_point);
//             	break;
// 		}	
//         printf("\nnext? (Yes:1,No:0) ");
// 		scanf("%d",&next);
// 		fflush(stdin);
//     } while (next);
// }

//ȷ���������,ֻ������ǰ180�����ڵ��ϰ��� 
void point_list::v_vector(struct Obs_point *head)
{
	struct Obs_point *p;
	//Ԥ������ 
	if(p->distance < warning_distance && p->distance > limit_distance)
	{
		//ȷ���Ƕ� 
		if(p->angle > 0.5*pi)
			out_point.ang = p->angle - 0.5*pi;
		else if(p->angle <= 0.5*pi)
			out_point.ang = p->angle + 0.5*pi;
		//�޷� 
		if(out_point.ang > angle_max)
			out_point.ang = angle_max;
		else if(out_point.ang <= angle_min)
			out_point.ang = angle_min;
		//ȷ�����루��λ���룩
		out_point.dis =  unit_distance;
		
	} 
	//���޾��� 
	else if(p->distance <= limit_distance && p->distance >= 0)
	{
		if(p->angle > 0.5*pi)
		{
			out_point.ang = angle_max;
			out_point.dis = unit_distance;
		}
		else if(p->angle <= 0.5*pi)
		{
			out_point.ang = angle_min;
			out_point.dis = unit_distance;
		}		
	}
}


struct Obs_point *point_list::creatlist()
{
	struct Obs_point *p1,*p2,*head=NULL;
	p2=p1=(struct Obs_point *)malloc(POINTLEN);
	p1->distance=LINK_HEAD_D;
	p1->angle=LINK_HEAD_A;
	head=p1;
	p2->next=NULL;
	return(head);
} 

void point_list::output(struct Obs_point *head)
{
	struct Obs_point *p;
	int num=1;
	p=head;;
	printf("|distance\t\t|angle\n");
	while(p!=NULL)
	{
		printf("%3d | %f\t | %f\n",num++,p->distance,p->angle);
		p=p->next;
	}
	printf("\n");
}

int point_list::append(struct Obs_point *head,float dis,float ang)
{
	struct Obs_point *p1,*p2;
	for(p1=head;p1->next!=NULL;p1=p1->next);//�ҵ�head���������һ��Ԫ�� 
	p2->next=p1;
	p2=p1;
	p1=(struct Obs_point *)malloc(POINTLEN);
	p1->distance = dis;
	p1->angle = ang;

	//û�����Ϊɶ����Ҫ��ôд 
	p2->next=p1;
	p2=p1;
	p1=(struct Obs_point *)malloc(POINTLEN);
	p1->distance = 0;
	p1->angle = 0;

	p2->next=NULL;
} 



//������������˱�ͷ 
struct Obs_point *point_list::clearlist(struct Obs_point *head)  
{  
    struct Obs_point *p,*q;  
    if(head==NULL)  
        return (head);  
    p=head->next;  
    while(p!=NULL)  
    {  
        q=p->next;  
        free(p);  
        p=q;  
    }  
    head->next=NULL;  
    return (head);  
} 



//�õ����������һ�����
struct Obs_point *point_list::getlastnode(struct Obs_point *head)
{
    struct Obs_point *p = head;
    while(p->next!=NULL)
    {
        p=p->next;
    }
    return p;
}
    
//ð������ 
struct Obs_point *point_list::sortlist(struct Obs_point *head)
{
    struct Obs_point *p1 = head;
    struct Obs_point *p2 = head;
    if(head == NULL)
        return NULL;
    for(;p1->next!=NULL;p1=p1->next)
    {
        for(p2=head;p2->next!=NULL;p2=p2->next)
        {
            if(p2->distance < p2->next->distance)
            {
            	//����                              //�Ƕ� 
                int temp_d = p2->distance; 			int temp_a = p2->angle;
                p2->distance = p2->next->distance; 	p2->angle = p2->next->angle;
                p2->next->distance = temp_d; 		p2->next->angle = temp_a;
            }
        }
    }
    return head;
}
