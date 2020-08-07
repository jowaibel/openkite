#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "kite_model/kite.hpp"
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/MultiDOFJointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "openkite/aircraft_controls.h"
#include <sensor_msgs/Joy.h>
#include <polympc/integration/integrator.h>

class Simulator
{
public:
    Simulator(const ODESolver &odeSolver, const ros::NodeHandle &nh);
    virtual ~Simulator(){}
    void simulate();

    casadi::DM getState(){return state;}
    casadi::DM getPose(){return state(casadi::Slice(6,13));}

    void publish_state(const ros::Time &sim_time);
    void publish_pose(const ros::Time &sim_time);

    bool is_initialized(){return initialized;}
    void initialize(const casadi::DM &_init_value){state = _init_value; initialized = true;}

    void setNumericVa(const casadi::Function &_NumericVa) {m_NumericVa = _NumericVa;}
    void setNumericAlpha(const casadi::Function &_NumericAlpha) {m_NumericAlpha = _NumericAlpha;}
    void setNumericBeta(const casadi::Function &_NumericBeta) {m_NumericBeta = _NumericBeta;}
    void setNumericVaPitot(const casadi::Function &_NumericVaPitot) {m_NumericVa_pitot = _NumericVaPitot;}
    void setNumericSpecNongravForce(const casadi::Function &_NumericSpecNongravForce) {m_NumericSpecNongravForce = _NumericSpecNongravForce;}
    void setNumericSpecTethForce(const casadi::Function &_NumericSpecTethForce) {m_NumericSpecTethForce = _NumericSpecTethForce;}
    void setNumericDebug(const casadi::Function &_NumericDebug) {m_NumericDebug = _NumericDebug;}


    bool sim_tether;

private:
    std::shared_ptr<ros::NodeHandle> m_nh;
    std::shared_ptr<ODESolver> m_odeSolver;
    casadi::Function m_NumericVa;
    casadi::Function m_NumericAlpha;
    casadi::Function m_NumericBeta;
    casadi::Function m_NumericVa_pitot;
    casadi::Function m_NumericSpecNongravForce;
    casadi::Function m_NumericSpecTethForce;
    casadi::Function m_NumericDebug;

    ros::Subscriber controlcmd_sub;
    ros::Publisher  state_pub;
    ros::Publisher  control_pub;
    ros::Publisher  tether_pub;
    ros::Publisher  pose_pub;

    casadi::DM      control_cmds;
    casadi::DM      state;
    double          Va_pitot{0};
    double          Va{0};
    double          alpha{0};
    double          beta{0};
    std::vector<double> specNongravForce{0,0,0};
    std::vector<double> specTethForce{0,0,0};

    void controlCallback(const sensor_msgs::JoyConstPtr &msg);
    double sim_rate;
    sensor_msgs::MultiDOFJointState msg_state;
    sensor_msgs::Joy                msg_control;
    geometry_msgs::Vector3Stamped msg_tether;

    bool initialized;
};

#endif // SIMULATOR_H
