#include<iostream>
#include<math.h>
using namespace std;
#define pi 3.1415926
int goal_x=0;
int goal_y=0;
int A=pow(10,9);
void fun1()
{
	int i1,i2,i3, a1, b1, a2, b2, a3, b3;
	cin>>i1>>i2>>i3>>a1>>b1>>a2>>b2>>a3>>b3;
    goal_x =
        (double)((pow(10, 9) * ((double)1 / (double)i1 - (double)1 / (double)i2) - (a1 * a1 + b1 * b1) + (a2 * a2 + b2 * b2)) * (-2 * b2 + 2 * b3) -
         (pow(10, 9) * ((double)1 / (double)i2 - (double)1 / (double)i3) - (a2 * a2 + b2 * b2) + (a3 * a3 + b3 * b3)) * (-2 * b1 + 2 * b2)) /(double)
        ((-2 * a1 + 2 * a2) * (-2 * b2 + 2 * b3) - (-2 * a2 + 2 * a3) * (-2 * b1 + 2 * b2));
    goal_y =
        (double)((pow(10, 9) * ((double)1 / (double)i1 - (double)1 / (double)i2) - (a1 * a1 + b1 * b1) + (a2 * a2 + b2 * b2)) * (-2 * a2 + 2 * a3) -
         (pow(10, 9) * ((double)1 / (double)i2 - (double)1 / (double)i3) - (a2 * a2 + b2 * b2) + (a3 * a3 + b3 * b3)) * (-2 * a1 + 2 * a2)) /(double)
        ((-2 * b1 + 2 * b2) * (-2 * a2 + 2 * a3) - (-2 * b2 + 2 * b3) * (-2 * a1 + 2 * a2));
} 
void fun2(int strength1,int strength2,int strength3,int x1,int y1,int x2,int y2,int x3,int y3)
{
	float k2 =(float)(y2-y1)/(float)(x2-x1);
	float k3 =(float)(y3-y1)/(float)(x3-x1);
	float p2 = (float)1/(float)k2;
		float p3 = (float)1/(float)k3;
		goal_y=((float)(A/strength3)/(float)(x3-x1)-(float)(A/strength2)/(float)(x2-x1)-(float)(x3-x2)+(float)k2*(float)(y1+y2)-(float)k3*(float)(y1+y3))/(float)(2*(k2-k3));
goal_x=((float)(A/strength3)/(float)(x3-x1)-(float)(A/strength2)/(float)(x2-x1)-(float)(y3-y2)+(float)p2*(x1+x2)-(float)p3*(x1+x3))/(float)(2*(p2-p3));
}
float coordinate_to_angle(int x, int y,int cur_x,int cur_y)
{
    int dx = x - cur_x;
    int dy = y - cur_y;
    double g_angle = 0;
    double delta = (double)dy / (double)dx;
    g_angle = (double) (atan(delta) * 180 / pi); // !!!HERE MAY EXIST BUGS!!!
    return -g_angle;
}
int main(){
	fun1();
	cout<<"goal_xc "<<goal_x<<" y:"<<goal_y<<endl;
//	fun2(78260,121758,94823,
//33,56
//,33,111,
//31,174);
//		cout<<"g0al_xc "<<goal_x<<" y:"<<goal_y<<endl;
cout<<coordinate_to_angle(102,202,76,47)<<endl;
}
