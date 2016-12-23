/*
* @Author: Trung Kien
* @Date:   2016-12-11 23:29:12
* @Last Modified by:   Kienltb
* @Last Modified time: 2016-12-23 11:10:32
*/

// #include "platform_test_case.h"
#include "balance_robot.h"

extern robot_t robot;

int main(int argc, char const *argv[])
{
        robot_init(&robot);
        robot_run(&robot);
        // robot_stop(&robot);
        return 0;
}
