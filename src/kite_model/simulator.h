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

class RandomGenerator
{
public:
    RandomGenerator()
    {
        distribution = std::uniform_real_distribution<double>(0, 1);
    }

    double getRand(const double &lower_bound, const double &upper_bound)
    {
        return lower_bound + distribution(generator) * (upper_bound - lower_bound);
    }
private:
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;
};

class DiscreteTurbulenceGenerator
{
public:
    DiscreteTurbulenceGenerator() = default;
    void init(const double &avg_wind_speed, const double &avg_wind_from)
    {
        V_avg = avg_wind_speed;
        V0 = V_avg;
        psiw_avg = avg_wind_from;
        psiw = psiw_avg;
    }
    void update(const double &t, const double &height,
                double &vWind_N, double &vWind_E)
    {
        double psiw_error = psiw_avg - psiw;
        if (psiw_error > M_PI) psiw_error -= 2 * M_PI;
        if (psiw_error < -M_PI) psiw_error += 2 * M_PI;

        const double dpsiw_max = 0.1;
        const double dpsiw = randGen.getRand(-dpsiw_max + psiw_error, dpsiw_max + psiw_error) * M_PI / 180.0;
        psiw = psiw + dpsiw;
        std::cout << "dpsiw: " << dpsiw * 180 / M_PI << " psiw: " << psiw * 180 / M_PI << "\n";

        if (gust_isArmed)
        {
            if (gust_gotArmed)
            {
                std::cout << "t = " << t_gust << "s. Gust armed\n";
                gust_gotArmed = false;
            }

            if (t_gust < t and t <= t_gust_end)
            {
                if (V_gust >= V_avg)
                    V = V0 + std::abs(V_gust - V0) / 2.0 * (1 - cos(M_PI * (t - t_gust) / (t_gust_end - t_gust)));
                else
                    V = V0 + std::abs(V_gust - V0) / 2.0 * (cos(M_PI * (t - t_gust) / (t_gust_end - t_gust)) - 1);
//                std::cout << "V: " << V << "\n";
            }
            else if (t > t_gust_end)
            {
                gust_isArmed = false;
                V0 = V;
            }
        }
        else
        {
            std::cout << "Determining next gust\n";
            /* If there is no gust, determine next one */
            /* Next gust will be in [0 ... 5] s */
            const double t_gust_in = randGen.getRand(0, 1);
            t_gust = t + t_gust_in;
            const double d_gust = randGen.getRand(0.5, 1);
            t_gust_end = t_gust + d_gust;

            const double a_gust_max = 0.3;
            const double dV_gust_max = d_gust * a_gust_max;
            const double V_avg_err = V_avg - V0;
            const double dV_gust = randGen.getRand(-dV_gust_max + V_avg_err, 2 * dV_gust_max + V_avg_err);
            V_gust = std::max(V0 + dV_gust, 0.0);

            gust_isArmed = true;
            gust_gotArmed = true;

            std::cout << "t_gust: " << t_gust << " d_gust: " << d_gust << " t_gust_end: " << t_gust_end << "\n"
                      << "dV_gust_max: " << dV_gust_max << "\n"
                      << "dV_gust: " << dV_gust << "\n"
                      << "V_gust: " << V_gust << "\n";
        }
        double V_cz = V * std::pow(height / 3.0, 0.1);
        vWind_N = V_cz * -cos(psiw);
        vWind_E = V_cz * -sin(psiw);

//        std::cout << "V: " << V << "\n"
//                  << "V_cz: " << V_cz << "\n"
//                  << "dpsi: " << dpsiw * 180.0 / M_PI << " deg\n"
//                  << "psi: " << psiw * 180.0 / M_PI << " deg\n";
        double stophere = 1;

    }

private:
    bool gust_isArmed{false};
    bool gust_gotArmed{false};
    double V_avg{1};
    double V0{V_avg};
    double V{V_avg};
    double psiw_avg{0};
    double psiw{psiw_avg};

    double V_gust{V_avg};
    double t_gust{0};
    double t_gust_end{t_gust};

    RandomGenerator randGen{};
};

class Simulator
{
public:
    Simulator(const ODESolver &odeSolver, const ros::NodeHandle &nh);
    virtual ~Simulator() {}
    void simulate();

    casadi::DM getState() { return state; }
    casadi::DM getPose() { return state(casadi::Slice(6, 13)); }

    void publish_state(const ros::Time &sim_time);
    void publish_pose(const ros::Time &sim_time);

    bool is_initialized() { return initialized; }
    void initialize(const casadi::DM &_init_value)
    {
        state = _init_value;
        initialized = true;
    }

    void setNumericVa(const casadi::Function &_NumericVa) { m_NumericVa = _NumericVa; }
    void setNumericAlpha(const casadi::Function &_NumericAlpha) { m_NumericAlpha = _NumericAlpha; }
    void setNumericBeta(const casadi::Function &_NumericBeta) { m_NumericBeta = _NumericBeta; }
    void setNumericVaPitot(const casadi::Function &_NumericVaPitot) { m_NumericVa_pitot = _NumericVaPitot; }
    void setNumericSpecNongravForce(
            const casadi::Function &_NumericSpecNongravForce) { m_NumericSpecNongravForce = _NumericSpecNongravForce; }
    void setNumericSpecTethForce(
            const casadi::Function &_NumericSpecTethForce) { m_NumericSpecTethForce = _NumericSpecTethForce; }
    void setNumericDebug(const casadi::Function &_NumericDebug) { m_NumericDebug = _NumericDebug; }

//    double vW_N{0};
//    double vW_E{0};
    bool sim_tether;
//    double wind_from_mean{0};
//    double wind_speed_mean{0};
    double sim_dt;
    DiscreteTurbulenceGenerator discreteTurbulenceGenerator;

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
    ros::Publisher state_pub;
    ros::Publisher control_pub;
    ros::Publisher tether_pub;
    ros::Publisher pose_pub;

    casadi::DM control_cmds;
    casadi::DM state;
    double Va_pitot{0};
    double Va{0};
    double alpha{0};
    double beta{0};
    std::vector<double> specNongravForce{0, 0, 0};
    std::vector<double> specTethForce{0, 0, 0};

    double sim_time{0};

    void controlCallback(const sensor_msgs::JoyConstPtr &msg);
    double sim_rate;

    sensor_msgs::MultiDOFJointState msg_state;
    sensor_msgs::Joy msg_control;
    geometry_msgs::Vector3Stamped msg_tether;

    bool initialized;
};

#endif // SIMULATOR_H
