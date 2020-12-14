#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "kite_model/kite.hpp"
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/MultiDOFJointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "openkite/aircraft_controls.h"
#include <sensor_msgs/Joy.h>
#include <polympc/integration/integrator.h>
#include "wind-dynamics/dryden_model.h"

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

/* DiscreteGustGenerator is strongly inspired by
 * Viktor, V. & Valery, I. & Yuri, A. & Shapovalov, Igor & Beloglazov, Denis. (2015). Simulation of wind effect on a
 * quadrotor flight. 10. 1535-1538.
 * with some modifications. */
class DiscreteGustGenerator
{
public:
    DiscreteGustGenerator() = default;
    void init(const double &avg_wind_speed, const double &avg_wind_from,
              const double &avg_gust_freq = 1.5, const double &max_dpsi = 0.2 * M_PI / 180)
    {
        V_avg = avg_wind_speed;
        psiw_avg = avg_wind_from;

        mean_gust_freq = avg_gust_freq;
        dpsiw_max = max_dpsi;

        V0 = V_avg;
        psiw = psiw_avg;

//        V_avg_val = V_avg;
//        psiw_avg_val = psiw_avg;
    }

    double mean_gust_freq{1.5};
    double dpsiw_max{0.3 * M_PI / 180.0};

    void update(const double &t, const double &height,
                double &vWind_N, double &vWind_E)
    {
        /* Wind (from) direction */
        /* Error from mean direction */
        double psiw_error = psiw_avg - psiw;
        if (psiw_error > M_PI) psiw_error -= 2 * M_PI;
        if (psiw_error < -M_PI) psiw_error += 2 * M_PI;

        const double dpsiw = randGen.getRand(-dpsiw_max + 0 * psiw_error, dpsiw_max + 0 * psiw_error);
        psiw = psiw + dpsiw;

        /* Wind speed */
        if (gust_isArmed)
        {
            if (gust_gotArmed) { gust_gotArmed = false; }

            if (t_gust < t and t <= t_gust_end)
            {
                if (V_gust >= V0)
                    V = V0 + std::abs(V_gust - V0) / 2.0 * (1 - cos(M_PI * (t - t_gust) / (t_gust_end - t_gust)));
                else
                    V = V0 + std::abs(V_gust - V0) / 2.0 * (cos(M_PI * (t - t_gust) / (t_gust_end - t_gust)) - 1);
            }
            else if (t > t_gust_end)
            {
                gust_isArmed = false;
                V0 = V;
//                std::cout << "V0 = " << V0 << "\n";
            }
        }
        else
        {
            /* If there is no gust, determine next one */
//            std::cout << "Determining next gust\n";
            const double t_gust_in = randGen.getRand(0, 2.0 / mean_gust_freq);
            t_gust = t + t_gust_in;
            const double d_gust = randGen.getRand(0, 2.0 / mean_gust_freq);
            t_gust_end = t_gust + d_gust;

            const double a_gust_max = 2.0; // Max. gust acceleration
            const double V_gust_max = 1.3 * V_avg;
            const double V_gust_min = 0.7 * V_avg;
            const double V_avg_err = V_avg - V0; // Error from mean speed

            const double dV_gust_max = std::min(d_gust * a_gust_max + V_avg_err, V_gust_max - V0);
            const double dV_gust_min = std::max(d_gust * -a_gust_max + V_avg_err, V_gust_min - V0);
            const double dV_gust = randGen.getRand(dV_gust_min, dV_gust_max);
            V_gust = std::max(V0 + dV_gust, 0.0);

            gust_isArmed = true;
            gust_gotArmed = true;

//            std::cout << "t_gust: " << t_gust << " d_gust: " << d_gust << " t_gust_end: " << t_gust_end << "\n"
//                      << "V_avg_err: " << V_avg_err << "\n"
//                      << "dV_gust_min: " << dV_gust_min << " dV_gust_max: " << dV_gust_max << "\n"
//                      << "dV_gust: " << dV_gust << "\n"
//                      << "V_gust: " << V_gust << "\n";
        }

        /* Wind velocity at aircraft height */
        const double V_cz = V * std::pow(height / 3.0, 0.1);
        vWind_N = V_cz * -cos(psiw);
        vWind_E = V_cz * -sin(psiw);

//        { /* Statistic mean values for validation */
//            V_avg_val = (V_avg_val * (avg_it - 1) + V_cz) / avg_it;
//            psiw_avg_val = (psiw_avg_val * (avg_it - 1) + psiw) / avg_it;
//            avg_it++;
//            std::cout << "V_cz: " << V_cz << " psiw: " << psiw
//                      << " V_avg_val: " << V_avg_val << " "
//                      << "psiw_avg_val: " << psiw_avg_val << "\n";
//        }
    }

private:
    RandomGenerator randGen{};

    bool gust_isArmed{false};
    bool gust_gotArmed{false};

    /* Average wind speed and orientation */
    double V_avg{1};
    double psiw_avg{0};

    /* Base wind speed V0, gust transition speed V, orientation psiw */
    double V0{V_avg};
    double V{V_avg};
    double psiw{psiw_avg};

    /* Times of arrival and end of next gust t_gust, t_gust_end, target wind speed of gust V_gust */
    double t_gust{0};
    double t_gust_end{t_gust};
    double V_gust{V_avg};

//    /* Long term average values (for validation) */
//    unsigned avg_it{1};
//    double V_avg_val{V_avg};
//    double psiw_avg_val{psiw_avg};

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

    void setNumericThrust(const casadi::Function &_NumericThrust) { m_NumericThrust = _NumericThrust; }
    void setNumericVa(const casadi::Function &_NumericVa) { m_NumericVa = _NumericVa; }
    void setNumericAlpha(const casadi::Function &_NumericAlpha) { m_NumericAlpha = _NumericAlpha; }
    void setNumericBeta(const casadi::Function &_NumericBeta) { m_NumericBeta = _NumericBeta; }
    void setNumericVaPitot(const casadi::Function &_NumericVaPitot) { m_NumericVa_pitot = _NumericVaPitot; }
    void setNumericSpecNongravForce(
            const casadi::Function &_NumericSpecNongravForce) { m_NumericSpecNongravForce = _NumericSpecNongravForce; }
    void setNumericSpecTethForce(
            const casadi::Function &_NumericSpecTethForce) { m_NumericSpecTethForce = _NumericSpecTethForce; }
    void setNumericDebug(const casadi::Function &_NumericDebug) { m_NumericDebug = _NumericDebug; }

    bool sim_tether;
    bool sim_turbulence;
    double Vw_N{0};
    double Vw_E{0};

    //    /* Long term mean values (for validation) */
    unsigned avg_it{1};
    double Vw_N_mean{Vw_N};
    double Vw_E_mean{Vw_E};

    double sim_dt;
    DiscreteGustGenerator discreteGustGenerator;
//    dryden_model::DrydenWind drydenWind;

private:
    std::shared_ptr<ros::NodeHandle> m_nh;
    std::shared_ptr<ODESolver> m_odeSolver;
    casadi::Function m_NumericThrust;
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
    ros::Publisher pose_px_pub;
    ros::Publisher twist_px_pub;
    ros::Publisher local_vel_px_pub;

    casadi::DM control_cmds;
    casadi::DM state;
    double thrust{0};
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
