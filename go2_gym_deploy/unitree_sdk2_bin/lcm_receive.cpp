#include <stdio.h>
#include <iostream>
#include <iomanip> 
#include <lcm/lcm-cpp.hpp>
#include "leg_control_data_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "rc_command_lcmt.hpp"


class Handler1
{
    public:
        ~Handler1() {}
//                const state_estimator_lcmt* state_estimator_data
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const leg_control_data_lcmt* leg_control_lcm_data)
        {
            int tab_space = 15; // 表格间隙
            std::cout << "**************** msgs name : leg_control_lcm_data ****************" << std::endl;
            std::cout << std::left << std::setw(tab_space) << "Motor id" << std::setw(tab_space) << "angle" << std::setw(tab_space) << "velocity"<< std::setw(tab_space) << "torque" << std::endl;
            for (int i = 0; i < 12; i++){
            std::cout << std::left << std::setw(tab_space) 
                        << i << std::setw(tab_space) 
                        << leg_control_lcm_data->q[i] << std::setw(tab_space) 
                        << leg_control_lcm_data->qd[i] << std::setw(tab_space) 
                        << leg_control_lcm_data->tau_est[i] << std::endl;
            }
            std::cout << std::endl;
        }
};


class Handler2
{
    public:
        ~Handler2() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const state_estimator_lcmt* state_estimator_data)
        {
            std::cout << "**************** msgs name: state_estimator_data ****************" << std::endl;
            std::cout << "quaternion: " << state_estimator_data->quat[0] << '\t'
                                        << state_estimator_data->quat[1] << '\t'
                                        << state_estimator_data->quat[2] << '\t'
                                        << state_estimator_data->quat[3] << '\t'<< std::endl;
            std::cout << "posture angles: " << std::endl 
                                            << "roll: "<< state_estimator_data->rpy[0] << '\t'
                                            << "pitch: "<< state_estimator_data->rpy[1] << '\t'
                                            << "yaw: "<< state_estimator_data->rpy[2] << '\t'<< std::endl;
            std::cout << "imu acc: " << std::endl 
                                            << "ax: "<< state_estimator_data->aBody[0] << '\t'
                                            << "ay: "<< state_estimator_data->aBody[1] << '\t'
                                            << "az: "<< state_estimator_data->aBody[2] << '\t'<< std::endl;
            std::cout << "imu omega: " << std::endl 
                                            << "wx: "<< state_estimator_data->omegaBody[0] << '\t'
                                            << "wy: "<< state_estimator_data->omegaBody[1] << '\t'
                                            << "wa: "<< state_estimator_data->omegaBody[2] << '\t'<< std::endl;
            std::cout << "foot force:  " << std::endl 
                                            << "FR foot force: "<< state_estimator_data->contact_estimate[0] << std::endl
                                            << "FL foot force: "<< state_estimator_data->contact_estimate[1] << std::endl
                                            << "RR foot force: "<< state_estimator_data->contact_estimate[2] << std::endl
                                            << "RL foot force: "<< state_estimator_data->contact_estimate[3] << std::endl<< std::endl;            
        }
};

class Handler3
{
    public:
        ~Handler3() {}
//                const state_estimator_lcmt* state_estimator_data
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const rc_command_lcmt* rc_command_data)
        {
            std::cout << "**************** msgs name: rc_command_data ****************" << std::endl;
            std::cout << "lx:  " << rc_command_data->left_stick[0] << std::endl;
            std::cout << "ly:  " << rc_command_data->left_stick[1] << std::endl;
            std::cout << "rx:  " << rc_command_data->right_stick[0] << std::endl;
            std::cout << "ry:  " << rc_command_data->right_stick[1] << std::endl;
            std::cout << "R1:  " << rc_command_data->right_upper_switch << std::endl;
            std::cout << "R2:  " << rc_command_data->right_lower_right_switch << std::endl;
            std::cout << "L1:  " << rc_command_data->left_upper_switch << std::endl;
            std::cout << "L2:  " << rc_command_data->left_lower_left_switch << std::endl;
            std::cout << "------------------------------------------------------------------------------------------" << std::endl;
        }
};


int main(int argc, char** argv)
{
    lcm::LCM lc;
    if(!lc.good()){
        std::cout << "lcm is error" << std::endl;
        return 1;
    }

    Handler1 handlerObject_leg_control_data;
    Handler2 handlerObject_state_estimator;
    Handler3 handlerObject_rc_command;


    lc.subscribe("leg_control_data", &Handler1::handleMessage, &handlerObject_leg_control_data);
    lc.subscribe("state_estimator_data", &Handler2::handleMessage, &handlerObject_state_estimator);
    lc.subscribe("rc_command_data", &Handler3::handleMessage, &handlerObject_rc_command);

    //  this handle function is essetial
    //  it waits for and dispatches messages
    //  Returns 0 on success, 
    //  Returns -1 if something went wrong.
    while(0 == lc.handle()){
        // do nothing
    };

    return 0;
}