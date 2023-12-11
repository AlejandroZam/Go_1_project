#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include "gaits.hpp"


using namespace UNITREE_LEGGED_SDK;
unitree_legged_msgs::LowState low_State;
unitree_legged_msgs::LowCmd low_cmd;
bool initiated_flag = false; // initiate need time
bool stand_flag = false;

enum State {
    WAIT,
  STANDUP,
  WALK,
  TROT,
  STANDSTILL,
    STANDTOWALK,
  RESET
  };

auto currentstate = WAIT;


void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{

    low_State.motorState[0].mode = msg->motorState[FR_0].mode;
    low_State.motorState[0].q = msg->motorState[FR_0].q;
    low_State.motorState[0].dq = msg->motorState[FR_0].dq;
    low_State.motorState[0].tauEst = msg->motorState[FR_0].tauEst;

    low_State.motorState[1].mode = msg->motorState[FR_1].mode;
    low_State.motorState[1].q = msg->motorState[FR_1].q;
    low_State.motorState[1].dq = msg->motorState[FR_1].dq;
    low_State.motorState[1].tauEst = msg->motorState[FR_1].tauEst;

    low_State.motorState[2].mode = msg->motorState[FR_2].mode;
    low_State.motorState[2].q = msg->motorState[FR_2].q;
    low_State.motorState[2].dq = msg->motorState[FR_2].dq;
    low_State.motorState[2].tauEst = msg->motorState[FR_2].tauEst;

    low_State.motorState[3].mode = msg->motorState[FL_0].mode;
    low_State.motorState[3].q = msg->motorState[FL_0].q;
    low_State.motorState[3].dq = msg->motorState[FL_0].dq;
    low_State.motorState[3].tauEst = msg->motorState[FL_0].tauEst;

    low_State.motorState[4].mode = msg->motorState[FL_1].mode;
    low_State.motorState[4].q = msg->motorState[FL_1].q;
    low_State.motorState[4].dq = msg->motorState[FL_1].dq;
    low_State.motorState[4].tauEst = msg->motorState[FL_1].tauEst;

    low_State.motorState[5].mode = msg->motorState[FL_2].mode;
    low_State.motorState[5].q = msg->motorState[FL_2].q;
    low_State.motorState[5].dq = msg->motorState[FL_2].dq;
    low_State.motorState[5].tauEst = msg->motorState[FL_2].tauEst;

    low_State.motorState[6].mode = msg->motorState[RR_0].mode;
    low_State.motorState[6].q = msg->motorState[RR_0].q;
    low_State.motorState[6].dq = msg->motorState[RR_0].dq;
    low_State.motorState[6].tauEst = msg->motorState[RR_0].tauEst;

    low_State.motorState[7].mode = msg->motorState[RR_1].mode;
    low_State.motorState[7].q = msg->motorState[RR_1].q;
    low_State.motorState[7].dq = msg->motorState[RR_1].dq;
    low_State.motorState[7].tauEst = msg->motorState[RR_1].tauEst;

    low_State.motorState[8].mode = msg->motorState[RR_2].mode;
    low_State.motorState[8].q = msg->motorState[RR_2].q;
    low_State.motorState[8].dq = msg->motorState[RR_2].dq;
    low_State.motorState[8].tauEst = msg->motorState[RR_2].tauEst;

    low_State.motorState[9].mode = msg->motorState[RL_0].mode;
    low_State.motorState[9].q = msg->motorState[RL_0].q;
    low_State.motorState[9].dq = msg->motorState[RL_0].dq;
    low_State.motorState[9].tauEst = msg->motorState[RL_0].tauEst;
    
    low_State.motorState[10].mode = msg->motorState[RL_1].mode;
    low_State.motorState[10].q = msg->motorState[RL_1].q;
    low_State.motorState[10].dq = msg->motorState[RL_1].dq;
    low_State.motorState[10].tauEst = msg->motorState[RL_1].tauEst;

    low_State.motorState[11].mode = msg->motorState[RL_2].mode;
    low_State.motorState[11].q = msg->motorState[RL_2].q;
    low_State.motorState[11].dq = msg->motorState[RL_2].dq;
    low_State.motorState[11].tauEst = msg->motorState[RL_2].tauEst;

    low_State.imu.quaternion[0] = msg->imu.quaternion[0];
    low_State.imu.quaternion[1] = msg->imu.quaternion[1];
    low_State.imu.quaternion[2] = msg->imu.quaternion[2];
    low_State.imu.quaternion[3] = msg->imu.quaternion[3];

    low_State.imu.gyroscope[0] = msg->imu.gyroscope[0];
    low_State.imu.gyroscope[1] = msg->imu.gyroscope[1];
    low_State.imu.gyroscope[2] = msg->imu.gyroscope[2];
    
    low_State.imu.accelerometer[0] = msg->imu.accelerometer[0];
    low_State.imu.accelerometer[1] = msg->imu.accelerometer[1];
    low_State.imu.accelerometer[2] = msg->imu.accelerometer[2];

 
    //FR
    low_State.footForce[0] = msg->footForce[0];

    //FL
    low_State.footForce[1] = msg->footForce[1];

    //RR
    low_State.footForce[2] = msg->footForce[2];

    //RL
    low_State.footForce[3] = msg->footForce[3];
}


float degtorad(double deg)
{

    float res = 0;

    res = (float)(deg * (3.14159/180));


    return res;
}

double radtodeg(double rad)
{
    return rad * (180/3.14159);
}

void paramInit()
{


low_cmd.head[0] = 0xFE;
low_cmd.head[1] = 0xEF;
low_cmd.levelFlag = 0xFF;   // LOWLEVEL;



for (int i = 0; i <12; i++) {
    low_cmd.motorCmd[i].mode = 0x0A;    // motor switch to servo (PMSM) mode
    low_cmd.motorCmd[i].q = (2.146E+9f);   // PosStopF; 
    low_cmd.motorCmd[i].Kp = 0;
    low_cmd.motorCmd[i].dq = (16000.0f);   // VelStopF; 
    low_cmd.motorCmd[i].Kd = 0;
    low_cmd.motorCmd[i].tau = 0;
}

    // printf("init here\n");
    // for(int i=0; i<2; i++){

    //     //Front
    //     //hip
    //     low_cmd.motorCmd[i*3+0].mode = 0x0A;
    //     low_cmd.motorCmd[i*3+0].Kp = 5;//46
    //     low_cmd.motorCmd[i*3+0].dq = 0;
    //     low_cmd.motorCmd[i*3+0].Kd = 3;//5
    //     low_cmd.motorCmd[i*3+0].tau = 0.0;
    //     // thigh
    //     low_cmd.motorCmd[i*3+1].mode = 0x0A;
    //     low_cmd.motorCmd[i*3+1].Kp = 15;//65
    //     low_cmd.motorCmd[i*3+1].dq = 0;
    //     low_cmd.motorCmd[i*3+1].Kd = 3;//4
    //     low_cmd.motorCmd[i*3+1].tau = 1.5;
    //     // calf
    //     low_cmd.motorCmd[i*3+2].mode = 0x0A;
    //     low_cmd.motorCmd[i*3+2].Kp = 15;//80
    //     low_cmd.motorCmd[i*3+2].dq = 0;
    //     low_cmd.motorCmd[i*3+2].Kd = 3;//6
    //     low_cmd.motorCmd[i*3+2].tau = 1.5;




    //     //back
    //     //hip
    //     low_cmd.motorCmd[i*3+6].mode = 0x0A;
    //     low_cmd.motorCmd[i*3+6].Kp = 5;//46
    //     low_cmd.motorCmd[i*3+6].dq = 0;
    //     low_cmd.motorCmd[i*3+6].Kd = 3;//5
    //     low_cmd.motorCmd[i*3+6].tau = 0.0;
    //     // thigh
    //     low_cmd.motorCmd[i*3+7].mode = 0x0A;
    //     low_cmd.motorCmd[i*3+7].Kp = 40;//65
    //     low_cmd.motorCmd[i*3+7].dq = 0;
    //     low_cmd.motorCmd[i*3+7].Kd = 5;//4
    //     low_cmd.motorCmd[i*3+7].tau = 1.6;
    //     // calf
    //     low_cmd.motorCmd[i*3+8].mode = 0x0A;
    //     low_cmd.motorCmd[i*3+8].Kp = 40;//80
    //     low_cmd.motorCmd[i*3+8].dq = 0;
    //     low_cmd.motorCmd[i*3+8].Kd = 4;//6
    //     low_cmd.motorCmd[i*3+8].tau = 1.6;
    // }
    // // for(int i=0; i<12; i++){
    // //     low_cmd.motorCmd[i].q = low_State.motorState[i].q;
    // //     printf("pos %f \n",low_cmd.motorCmd[i].q);
    // // }
    // low_cmd.motorCmd[FR_0].tau = +0.2f;
    // low_cmd.motorCmd[FL_0].tau = +0.2f;
    // low_cmd.motorCmd[RR_0].tau = +0.2f;
    // low_cmd.motorCmd[RL_0].tau = +0.2f;

    // low_cmd.motorCmd[FR_0].q = +0.0;
    // low_cmd.motorCmd[FL_0].q = +0.0;
    // low_cmd.motorCmd[RR_0].q = +0.0;
    // low_cmd.motorCmd[RL_0].q = +0.0;


    // low_cmd.motorCmd[FR_1].q = degtorad(18.0);
    // low_cmd.motorCmd[FL_1].q = degtorad(18.0);
    // low_cmd.motorCmd[RR_1].q = degtorad(20.0);
    // low_cmd.motorCmd[RL_1].q = degtorad(20.0);


    // low_cmd.motorCmd[FR_2].q = degtorad(-75.0);
    // low_cmd.motorCmd[FL_2].q = degtorad(-75.0);
    // low_cmd.motorCmd[RR_2].q = degtorad(-70.0);
    // low_cmd.motorCmd[RL_2].q = degtorad(-70.0);






    // initiated_flag = true;
    // printf("init done\n");
}



void standup()
{
 









}




std::pair<double,double> get_ik(double x, double y)
{

    gaitlib::IKResult cur_q{0.0,0.0,0.0,0.0};

    cur_q = gaitlib::ik(x,y);

    std::pair<double,double> result = make_pair(0.0,0.0);


    if (gaitlib::THIGH_LO < cur_q.thigh_lefty && cur_q.thigh_lefty < gaitlib::THIGH_HI )
    {
        result.first = cur_q.thigh_lefty;
    }
    // else{
    //     result.first = cur_q.thigh_righty;
    // }
    else if(gaitlib::THIGH_LO < cur_q.thigh_righty && cur_q.thigh_righty < gaitlib::THIGH_HI )
    {
        result.first = NAN;
    }
    else
    {
        result.first = low_State.motorState[1].q;
    }
    if (gaitlib::CALF_LO < cur_q.calf_lefty && cur_q.calf_lefty < gaitlib::CALF_HI )
    {
        result.second = cur_q.calf_lefty;
    }
    // else
    // {
    //     result.second = cur_q.calf_righty;
    // }
    else if(gaitlib::CALF_LO < cur_q.calf_righty && cur_q.calf_righty < gaitlib::CALF_HI )
    {
        result.second = cur_q.calf_righty;
    }
    else
    {
        result.second = NAN;
    }

    return result;
}








int main(int argc, char **argv)
{
    ros::init(argc, argv, "walk");

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;
      double rate_hz, stroke_length_x, stroke_length_z, standup_time, stiffness, damping, delta,
         stand_percentage, step_height, dx1, dx2, dz1, dz2, dy;
    standup_time = 2.0;
    rate_hz = 200;
    ros::Rate loop_rate(rate_hz);

    int motion_step = 0;

    int sin_time_step = 0;
    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    std::pair<double,double> thetas = make_pair(0.0,0.0);
    gaitlib::IKResult cur_q{0.0,0.0,0.0,0.0};

    double thigh_q = 0.0;
    double calf_q = 0.0;
    int step_to_stand = 11;
    
    // unitree_legged_msgs::low_cmd low_cmd;
 
    // bool initiated_flag = false; // initiate need time
    long time_step = 0;
    int temp_timer = 0;
    ros::Publisher pub = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
    ros::Subscriber low_sub = nh.subscribe("low_state", 1, lowStateCallback);
    bool step_limit;
    step_limit = true;
    double stand_y = -2.0 * 0.8 * gaitlib::LEG_LENGTH;
    double stand_x = -0.05;
    double stand_z = 0;
    double stand_calf, stand_thigh;
    paramInit();
      std::vector<double> ctrl_x, ctrl_y, ctrl_z;
  std::vector<double> fr_calf_walk, fl_calf_walk, rr_calf_walk, rl_calf_walk,
    fr_thigh_walk, fl_thigh_walk, rr_thigh_walk, rl_thigh_walk,
    fr_hip_walk, fl_hip_walk, rr_hip_walk, rl_hip_walk;

  std::vector<double> fr_calf_switch, fl_calf_switch, rr_calf_switch, rl_calf_switch,
    fr_thigh_switch, fl_thigh_switch, rr_thigh_switch, rl_thigh_switch;


    std::vector<double> fr_calf_stand, fl_calf_stand, rr_calf_stand, rl_calf_stand,
        fr_thigh_stand, fl_thigh_stand, rr_thigh_stand, rl_thigh_stand;

    const auto stand_up_x1 = gaitlib::linspace(0.0,0.0,rate_hz);
    const auto stand_up_y1 = gaitlib::linspace(-0.1, -0.1, rate_hz);

    const auto stand_up_x2 = gaitlib::linspace(0.0, stand_x, rate_hz*standup_time);
    const auto stand_up_y2 = gaitlib::linspace(-0.1, stand_y, rate_hz*standup_time);


    const auto stand_up_x = gaitlib::concatenate(stand_up_x1, stand_up_x2);
    const auto stand_up_y = gaitlib::concatenate(stand_up_y1, stand_up_y2);

    const auto stand_up_gait = gaitlib::make_gait(stand_up_x, stand_up_y);

    fr_calf_stand = stand_up_gait.gait_calf;
    fr_thigh_stand = stand_up_gait.gait_thigh;
    fl_calf_stand = stand_up_gait.gait_calf;
    fl_thigh_stand = stand_up_gait.gait_thigh;
    rr_calf_stand = stand_up_gait.gait_calf;
    rr_thigh_stand = stand_up_gait.gait_thigh;
    rl_calf_stand = stand_up_gait.gait_calf;
    rl_thigh_stand = stand_up_gait.gait_thigh;


    const auto standing_joints = gaitlib::ik(stand_x, stand_y);
    stand_calf = standing_joints.calf_lefty;
    stand_thigh = standing_joints.thigh_lefty;

    //walk

    double seconds_per_swing = 0.25;
    //walk
    seconds_per_swing = 0.25;



    stroke_length_x = 0.05;
    stroke_length_z = 0.025;
    // stroke_length_z = 0.03;
    long period = rate_hz *seconds_per_swing;

    stiffness = 90.0;
    damping = 5.0;
    delta = 0.05;
    // delta = 0.01;
    stand_percentage = 0.75;

    step_height = 0.05;
    dx1 = 0.025;
    dx2 = 0.025;
    dz1 = 0.025;
    dz2 = 0.025;
    dy = 0.025;
      // this is the bezier curve of feet position
    std::vector<double> bez_x, bez_y, bez_z;
    int tripod_offset, trot_offset,walk_offset, nsteps, step_count;
    trot_offset = 0;
    walk_offset = 0;
    nsteps = 20;


    const auto lspan_x = 0.5 * stroke_length_x;   // half of "stroke length", ie how long it's on the floor
    const auto lspan_z = 0.5 * stroke_length_z;
    ctrl_x = std::vector<double> {
      stand_x + -1.0 * lspan_x,
      stand_x + -1.0 * lspan_x - dx1,
      stand_x + -1.0 * lspan_x - dx1 - dx2,
      stand_x + -1.0 * lspan_x - dx1 - dx2,
      stand_x + -1.0 * lspan_x - dx1 - dx2,
      stand_x + 0.0,
      stand_x + 0.0,
      stand_x + 0.0,
      stand_x + lspan_x + dx1 + dx2,
      stand_x + lspan_x + dx1 + dx2,
      stand_x + lspan_x + dx1,
      stand_x + lspan_x};
    ctrl_y = std::vector<double> {
      stand_y,
      stand_y,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height,
      stand_y + step_height + dy,
      stand_y + step_height + dy,
      stand_y + step_height + dy,
      stand_y,
      stand_y};
    // for z controls (valid for sideways only) I just make it basically same as x.
    ctrl_z = std::vector<double> {
      stand_z + -1.0 * lspan_z,
      stand_z + -1.0 * lspan_z - dz1,
      stand_z + -1.0 * lspan_z - dz1 - dz2,
      stand_z + -1.0 * lspan_z - dz1 - dz2,
      stand_z + -1.0 * lspan_z - dz1 - dz2,
      stand_z + 0.0,
      stand_z + 0.0,
      stand_z + 0.0,
      stand_z + lspan_z + dz1 + dz2,
      stand_z + lspan_z + dz1 + dz2,
      stand_z + lspan_z + dz1,
      stand_z + lspan_z};


    // generate the curve 
    bez_x = gaitlib::bezier(ctrl_x, period);
    bez_y = gaitlib::bezier(ctrl_y, period);
    bez_z = gaitlib::bezier(ctrl_z, period);




    // trot works
    long npoints_wait = trot_offset*period;

    std::vector<double> final_x, final_y;

    if (trot_offset == 0){
      // this corresponds to constant motion
      const auto ctrl_back = ctrl_x.back();
      const auto ctrl_front = ctrl_x.front();

      std::vector<double> sin_x = gaitlib::linspace(ctrl_back, ctrl_front, period);
      std::vector<double> sin_y = gaitlib::stance(sin_x, delta, ctrl_y.at(0));

      final_x = gaitlib::concatenate(bez_x, sin_x);
      final_y = gaitlib::concatenate(bez_y, sin_y);
    } else {
      // aitin period 
      std::vector<double> sin_x =
        gaitlib::linspace(ctrl_x.at(0), ctrl_x.at(0), period);
      std::vector<double> sin_y =
        gaitlib::stance(sin_x, delta, ctrl_y.at(0));

      const auto moving_x = gaitlib::concatenate(sin_x, bez_x);
      const auto moving_y = gaitlib::concatenate(sin_y, bez_y);

      const auto wait_x = gaitlib::linspace(moving_x.back(), moving_x.at(0), npoints_wait);
      const auto wait_y = gaitlib::linspace(moving_y.back(), moving_y.at(0), npoints_wait);

      final_x = gaitlib::concatenate(moving_x, wait_x);
      final_y = gaitlib::concatenate(moving_y, wait_y);
    }


    const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    fr_calf_walk = fr_gaits.gait_calf;
    fr_thigh_walk = fr_gaits.gait_thigh;
    // hip in this walk is just constant at 0
    fr_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);
    // Next: MODULATE based on fr
    fl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    fl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);
    fl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);

    rr_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    rr_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);
    rr_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);

    rl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.0);
    rl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.0);
    rl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);
    //works

    //walk

    // walk_offset = 5.0;
    // long npoints_wait = walk_offset*period;

    // std::vector<double> final_x, final_y;
    
    // if (walk_offset == 0){
    //   // this corresponds to constant motion
    //   const auto ctrl_back = ctrl_x.back();
    //   const auto ctrl_front = ctrl_x.front();

    //   std::vector<double> sin_x = gaitlib::linspace(ctrl_back, ctrl_front, period);
    //   std::vector<double> sin_y = gaitlib::stance(sin_x, delta, ctrl_y.at(0));

    //   final_x = gaitlib::concatenate(bez_x, sin_x);
    //   final_y = gaitlib::concatenate(bez_y, sin_y);
    // } else {
    //   // aitin period 
    //   std::vector<double> sin_x =
    //     gaitlib::linspace(ctrl_x.at(0), ctrl_x.at(0), period);
    //   std::vector<double> sin_y =
    //     gaitlib::stance(sin_x, delta, ctrl_y.at(0));

    //   const auto moving_x = gaitlib::concatenate(sin_x, bez_x);
    //   const auto moving_y = gaitlib::concatenate(sin_y, bez_y);

    //   const auto wait_x = gaitlib::linspace(moving_x.back(), moving_x.at(0), npoints_wait);
    //   const auto wait_y = gaitlib::linspace(moving_y.back(), moving_y.at(0), npoints_wait);

    //   final_x = gaitlib::concatenate(moving_x, wait_x);
    //   final_y = gaitlib::concatenate(moving_y, wait_y);
    // }


    // const auto fl_gaits = gaitlib::make_gait(final_x, final_y);
    // fl_calf_walk = fl_gaits.gait_calf;
    // fl_thigh_walk = fl_gaits.gait_thigh;
    // // hip in this walk is just constant at 0
    // fl_hip_walk = std::vector<double>(fl_calf_walk.size(), 0.0);
    // // Next: MODULATE based on fr
    // fr_calf_walk = gaitlib::modulate(fl_calf_walk, 0.5);
    // fr_thigh_walk = gaitlib::modulate(fl_thigh_walk, 0.5);
    // fr_hip_walk = gaitlib::modulate(fl_hip_walk, 0.5);

    // rr_calf_walk = gaitlib::modulate(fl_calf_walk, 0.75);
    // rr_thigh_walk = gaitlib::modulate(fl_thigh_walk, 0.75);
    // rr_hip_walk = gaitlib::modulate(fl_hip_walk, 0.75);

    // rl_calf_walk = gaitlib::modulate(fl_calf_walk, 0.25);
    // rl_thigh_walk = gaitlib::modulate(fl_thigh_walk, 0.25);
    // rl_hip_walk = gaitlib::modulate(fl_hip_walk, 0.25);







    // const auto fr_gaits = gaitlib::make_gait(final_x, final_y);
    // fr_calf_walk = fr_gaits.gait_calf;
    // fr_thigh_walk = fr_gaits.gait_thigh;
    // // hip in this walk is just constant at 0
    // fr_hip_walk = std::vector<double>(fr_calf_walk.size(), 0.0);
    // // Next: MODULATE based on fr
    // fl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.5);
    // fl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.5);
    // fl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.5);

    // rr_calf_walk = gaitlib::modulate(fr_calf_walk, 0.75);
    // rr_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.75);
    // rr_hip_walk = gaitlib::modulate(fr_hip_walk, 0.75);

    // rl_calf_walk = gaitlib::modulate(fr_calf_walk, 0.25);
    // rl_thigh_walk = gaitlib::modulate(fr_thigh_walk, 0.25);
    // rl_hip_walk = gaitlib::modulate(fr_hip_walk, 0.25);



    // switch code

    const auto foot_current = gaitlib::fk(stand_thigh, stand_calf);
    const auto fr_desired = gaitlib::fk(fr_thigh_walk.at(0), fr_calf_walk.at(0));
    const auto fl_desired = gaitlib::fk(fl_thigh_walk.at(0), fl_calf_walk.at(0));
    const auto rr_desired = gaitlib::fk(rr_thigh_walk.at(0), rr_calf_walk.at(0));
    const auto rl_desired = gaitlib::fk(rl_thigh_walk.at(0), rl_calf_walk.at(0));

    const auto fr_switch_x = gaitlib::linspace(foot_current.x, fr_desired.x, rate_hz*0.25);
    const auto fr_switch_y = gaitlib::linspace(foot_current.y, fr_desired.y, rate_hz*0.25);

    const auto fr_switch = gaitlib::make_gait(fr_switch_x, fr_switch_y);
    fr_calf_switch = fr_switch.gait_calf;
    fr_thigh_switch = fr_switch.gait_thigh;

    const auto fl_switch_x = gaitlib::linspace(foot_current.x, fl_desired.x, rate_hz*0.25);
    const auto fl_switch_y = gaitlib::linspace(foot_current.y, fl_desired.y, rate_hz*0.25);
    const auto fl_switch = gaitlib::make_gait(fl_switch_x, fl_switch_y);
    fl_calf_switch = fl_switch.gait_calf;
    fl_thigh_switch = fl_switch.gait_thigh;

    const auto rr_switch_x = gaitlib::linspace(foot_current.x, rr_desired.x, rate_hz*0.25);
    const auto rr_switch_y = gaitlib::linspace(foot_current.y, rr_desired.y, rate_hz*0.25);
    const auto rr_switch = gaitlib::make_gait(rr_switch_x, rr_switch_y);
    rr_calf_switch = rr_switch.gait_calf;
    rr_thigh_switch = rr_switch.gait_thigh;

    const auto rl_switch_x = gaitlib::linspace(foot_current.x, rl_desired.x, rate_hz*0.25);
    const auto rl_switch_y = gaitlib::linspace(foot_current.y, rl_desired.y, rate_hz*0.25);
    const auto rl_switch = gaitlib::make_gait(rl_switch_x, rl_switch_y);
    rl_calf_switch = rl_switch.gait_calf;
    rl_thigh_switch = rl_switch.gait_thigh;





    initiated_flag = true;

    printf("robot start\n");
    while (ros::ok())
    {

        switch(currentstate)
        {

            case WAIT:
            {
            printf("waiting\n");

            if (time_step > 10)
            {

            currentstate = STANDUP;

            // works
            for (int i = 0; i < 12; i++) {
              low_cmd.motorCmd[i].dq = 0.0;
              low_cmd.motorCmd[i].Kp = 90.0;
              low_cmd.motorCmd[i].Kd = 5.0;
              //tuning 

            }
            // works

                // low_cmd.motorCmd[FR_0].Kp =89.0;
                // low_cmd.motorCmd[FL_0].Kp =89.0;
                // low_cmd.motorCmd[RR_0].Kp =89.0;
                // low_cmd.motorCmd[RL_0].Kp =89.0;

                // low_cmd.motorCmd[FR_0].Kd = 2.0;
                // low_cmd.motorCmd[FL_0].Kd = 2.0;
                // low_cmd.motorCmd[RR_0].Kd = 2.0;
                // low_cmd.motorCmd[RL_0].Kd = 2.0;

                // low_cmd.motorCmd[FR_0].tau = -0.2f;
                // low_cmd.motorCmd[FL_0].tau = -0.2f;
                // low_cmd.motorCmd[RR_0].tau = -0.2f;
                // low_cmd.motorCmd[RL_0].tau = -0.2f;

    // for(int i=0; i<2; i++){

    //     //Front
    //     //hip
    //     low_cmd.motorCmd[i*3+0].Kp = 90;//46
    //     low_cmd.motorCmd[i*3+0].dq = 0.0;
    //     low_cmd.motorCmd[i*3+0].Kd = 5.0;//5
    //     // thigh
    //     low_cmd.motorCmd[i*3+1].Kp = 90.0;//65
    //     low_cmd.motorCmd[i*3+1].dq = 0.0;
    //     low_cmd.motorCmd[i*3+1].Kd = 5.0;//4
    //     // calf
    //     low_cmd.motorCmd[i*3+2].Kp = 90.0;//80
    //     low_cmd.motorCmd[i*3+2].dq = 0.0;
    //     low_cmd.motorCmd[i*3+2].Kd = 5.0;//6
    //     //back
    //     //hip
    //     low_cmd.motorCmd[i*3+6].Kp = 90;//46
    //     low_cmd.motorCmd[i*3+6].dq = 0.0;
    //     low_cmd.motorCmd[i*3+6].Kd = 5.0;//5
    //     // thigh
    //     low_cmd.motorCmd[i*3+7].Kp = 90.0;//65
    //     low_cmd.motorCmd[i*3+7].dq = 0.0;
    //     low_cmd.motorCmd[i*3+7].Kd = 5.0;//4
    //     // calf
    //     low_cmd.motorCmd[i*3+8].Kp = 90.0;//80
    //     low_cmd.motorCmd[i*3+8].dq = 0.0;
    //     low_cmd.motorCmd[i*3+8].Kd = 5.0;//6

    // }











           
            time_step = 0;
            }
            time_step++;
            break;
            }

            case STANDUP:
            printf("standing\n");
            {
            low_cmd.motorCmd[FR_0].q = 0.0;
            low_cmd.motorCmd[FR_1].q = fr_thigh_stand.at(time_step);
            low_cmd.motorCmd[FR_2].q =  fr_calf_stand.at(time_step);

                                
            low_cmd.motorCmd[FL_0].q =  0.0;
            low_cmd.motorCmd[FL_1].q =fl_thigh_stand.at(time_step);
            low_cmd.motorCmd[FL_2].q = fl_calf_stand.at(time_step);
                                  
            low_cmd.motorCmd[RR_0].q =  0.0;
            low_cmd.motorCmd[RR_1].q = rr_thigh_stand.at(time_step);
            low_cmd.motorCmd[RR_2].q = rr_calf_stand.at(time_step);
                                  
            low_cmd.motorCmd[RL_0].q =  0.0;
            low_cmd.motorCmd[RL_1].q = rl_thigh_stand.at(time_step);
            low_cmd.motorCmd[RL_2].q = rl_calf_stand.at(time_step);

            time_step++;

            if (time_step >= static_cast<long>(fr_calf_stand.size())) {
            time_step = 0;
            currentstate = STANDSTILL;
            }
            break;
            }

            
            case WALK:
            {
            printf("WALK\n");


            low_cmd.motorCmd[FR_0].q = fr_hip_walk.at(time_step);
            low_cmd.motorCmd[FR_1].q = fr_thigh_walk.at(time_step);
            low_cmd.motorCmd[FR_2].q =  fr_calf_walk.at(time_step);

                                
            low_cmd.motorCmd[FL_0].q =  fl_hip_walk.at(time_step);
            low_cmd.motorCmd[FL_1].q = fl_thigh_walk.at(time_step);
            low_cmd.motorCmd[FL_2].q = fl_calf_walk.at(time_step);
                                  
            low_cmd.motorCmd[RR_0].q =  rr_hip_walk.at(time_step);
            low_cmd.motorCmd[RR_1].q = rr_thigh_walk.at(time_step);
            low_cmd.motorCmd[RR_2].q = rr_calf_walk.at(time_step);
                                  
            low_cmd.motorCmd[RL_0].q =  rl_hip_walk.at(time_step);
            low_cmd.motorCmd[RL_1].q = rl_thigh_walk.at(time_step);
            low_cmd.motorCmd[RL_2].q = rl_calf_walk.at(time_step);


            time_step++;

            if (time_step >= static_cast<long>(fr_calf_walk.size())) {

            step_count++;
            time_step = 0;
            if (step_limit && (step_count >= nsteps)){

                currentstate = STANDSTILL;
                step_count = 0;
            }
            }

            break;
            }

            case STANDSTILL:
            {
            printf("STANDSTILL\n");

            low_cmd.motorCmd[FR_0].q = 0.0;
            low_cmd.motorCmd[FR_1].q = stand_thigh;
            low_cmd.motorCmd[FR_2].q =  stand_calf;

                                
            low_cmd.motorCmd[FL_0].q =  0.0;
            low_cmd.motorCmd[FL_1].q = stand_thigh;
            low_cmd.motorCmd[FL_2].q = stand_calf;
                                  
            low_cmd.motorCmd[RR_0].q =  0.0;
            low_cmd.motorCmd[RR_1].q = stand_thigh;
            low_cmd.motorCmd[RR_2].q = stand_calf;
                                  
            low_cmd.motorCmd[RL_0].q =  0.0;
            low_cmd.motorCmd[RL_1].q = stand_thigh;
            low_cmd.motorCmd[RL_2].q = stand_calf;

            break;
            }
            

            case STANDTOWALK:
            {
            printf("Standtowalk\n");


            low_cmd.motorCmd[FR_0].q = 0.0;
            low_cmd.motorCmd[FR_1].q = fr_thigh_switch.at(time_step);
            low_cmd.motorCmd[FR_2].q =  fr_calf_switch.at(time_step);

                                
            low_cmd.motorCmd[FL_0].q =  0.0;
            low_cmd.motorCmd[FL_1].q = fl_thigh_switch.at(time_step);
            low_cmd.motorCmd[FL_2].q = fl_calf_switch.at(time_step);
                                  
            low_cmd.motorCmd[RR_0].q =  0.0;
            low_cmd.motorCmd[RR_1].q = rr_thigh_switch.at(time_step);
            low_cmd.motorCmd[RR_2].q = rr_calf_switch.at(time_step);
                                  
            low_cmd.motorCmd[RL_0].q =  0.0;
            low_cmd.motorCmd[RL_1].q = rl_thigh_switch.at(time_step);
            low_cmd.motorCmd[RL_2].q = rl_calf_switch.at(time_step);


            time_step++;

            if (time_step >= static_cast<long>(fr_calf_switch.size())) {
            time_step = 0;
            currentstate = WALK;
            }
            break;
            }

            case RESET:
            {
            printf("RESET\n");
            for (int i = 0; i < 12; i++) {
            low_cmd.motorCmd[i].tau = 0.0;
            }
            break;
            }

        }

        if (currentstate == STANDSTILL )
        {
            temp_timer++;
        }

        if (temp_timer> 1000)
        {
            
            printf("walk the dog");
            currentstate = STANDTOWALK;
            temp_timer = 0;
        }

        pub.publish(low_cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}