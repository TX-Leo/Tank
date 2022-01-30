#include <iostream>
#include <math.h>
#define pi 3.1415926
using namespace std;
float coordinate_to_angle()
{
    int cur_x;
    int cur_y;
    int x;
    int y;
    cin >> cur_x >> cur_y >> x >> y;
    if (cur_x < x)
    {

        int dx = x - cur_x;
        int dy = y - cur_y;
        double g_angle = 0;
        double delta = (double)dy / (double)dx;
        g_angle = (float)(atan(delta) * 180 / pi); // !!!HERE MAY EXIST BUGS!!!
        if (g_angle < 0)
        {
            g_angle += 360;
        }
        return g_angle;
    }
    else
    {
        int dx = x - cur_x;
        int dy = y - cur_y;
        double g_angle = 0;
        double delta = (double)dy / (double)dx;
        g_angle = (float)(atan(delta) * 180 / pi)+180; // !!!HERE MAY EXIST BUGS!!!
        if (g_angle < 0)
        {
            g_angle += 360;
        }
        return g_angle;
    }
}

int main()
{
    cout << coordinate_to_angle() << endl;
    return 1;
}
