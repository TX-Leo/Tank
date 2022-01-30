#include<iostream>
#include<math.h>
using namespace std;
int cur_x=100;
int cur_y=0;
#define pi 3.1415926
double coordinate_to_angle(int x, int y)
{
    int dx = x - cur_x;
    int dy = y - cur_y;
    double delta=(double)dy/(double)dx;
    double g_angle = 0;
    g_angle =atan(delta)*180/pi; // !!!HERE MAY EXIST BUGS!!!
    return g_angle;
}
int main(){
	cout<<"angle:"<<coordinate_to_angle(30,140)<<endl;;
}

