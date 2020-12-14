#include "simulator.h"

using namespace casadi;

void Simulator::controlCallback(const sensor_msgs::JoyConstPtr &msg)
{
    control_cmds(0) = msg->axes[0]; // Thrust
    control_cmds(1) = msg->axes[1]; // Elevator
    control_cmds(2) = msg->axes[2]; // Rudder
    control_cmds(3) = msg->axes[3]; // Aileron
}

Simulator::Simulator(const ODESolver &odeSolver, const ros::NodeHandle &nh)
{
    m_odeSolver = std::make_shared<ODESolver>(odeSolver);
    m_nh = std::make_shared<ros::NodeHandle>(nh);

    /** define dimensions first given solver object */
    state = DM::zeros(m_odeSolver->dim_x());
    control_cmds = DM::zeros(m_odeSolver->dim_u());

    /** initialize subscribers and publishers */
    int broadcast_state;
    m_nh->param<int>("broadcast_state", broadcast_state, 1);

    /** initialize subscribers and publishers */
    m_nh->param<bool>("simulate_tether", sim_tether, false);

    std::vector<double> init_state;
    m_nh->getParam("init_state", init_state);
    /* Init values for Actuator positions */
//    init_state = {15, 0, 0,  0, 0, 0,  20, 5, -120, 0, 0, -135*M_PI/180.0}; // use Euler angle attitude representation
    init_state.emplace_back(0);
    init_state.emplace_back(0);
    init_state.emplace_back(0);
    init_state.emplace_back(0);
    initialize(DM(init_state));
    ROS_INFO_STREAM("Simulator initialized at: " << init_state);

    //pose_pub  = m_nh->advertise<geometry_msgs::PoseStamped>("/sim/kite_pose", 100);
    state_pub = m_nh->advertise<sensor_msgs::MultiDOFJointState>("/sim/kite_state", 1);
    control_pub = m_nh->advertise<sensor_msgs::Joy>("/sim/kite_controls", 1);
    tether_pub = m_nh->advertise<geometry_msgs::Vector3Stamped>("/sim/tether_force", 1);

    controlcmd_sub = m_nh->subscribe("/sim/set/kite_controls", 100, &Simulator::controlCallback, this);

    /** UNCOMMENT THIS FOR TESTING ONLY **/
    /* Additional publisher to simulate pixhawk (mavros) topics */
//    pose_px_pub = m_nh->advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1);
//    twist_px_pub = m_nh->advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 1);
//    local_vel_px_pub = m_nh->advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1);

    msg_state.twist.resize(2);
    msg_state.transforms.resize(1);
    msg_state.wrench.resize(2);

    msg_state.header.frame_id = "kite_sim";

    msg_control.axes.resize(4);
}

void Simulator::simulate()
{
    Dict p = m_odeSolver->getParams();
    double dt = p["tf"];

    /* Make sure thrust is not negative */
    if (static_cast<double>(control_cmds(0)) < 0.0) control_cmds(0) = 0.0;

    /* Update wind */
    Eigen::Vector3d wind_velocity{};
    if (sim_turbulence)
    {
        discreteGustGenerator.update(sim_time, -state(8).nonzeros()[0], Vw_N, Vw_E);
//        wind_velocity = drydenWind.getWind(sim_dt);
//        Vw_N = wind_velocity(0);
//        Vw_E = wind_velocity(1);

        {
//        /* Statistic mean values for validation */
//            Vw_N_mean = (Vw_N_mean * (avg_it - 1) + Vw_N) / avg_it;
//            Vw_E_mean = (Vw_E_mean * (avg_it - 1) + Vw_E) / avg_it;
//            avg_it++;
//
//            double windSpeed_val = sqrt(Vw_N_mean * Vw_N_mean + Vw_E_mean * Vw_E_mean);
//            std::cout << "Vw_N: " << Vw_N << " Vw_E: " << Vw_E
//                      << " Vw_N_mean: " << Vw_N_mean << " Vw_E_mean: " << Vw_E_mean
//                      << "  magnitude_mean: " << windSpeed_val << "\n";
        }
    }

    DM dyn_params = DM::vertcat({Vw_N, Vw_E});
    state = m_odeSolver->solve(state, control_cmds, dyn_params, dt);

    DM control_dummy;
    /* Get thrust */
    thrust = m_NumericThrust(DMVector{state, control_dummy, dyn_params})[0].nonzeros()[0];

    /* Get pitot airspeed */
    Va_pitot = m_NumericVa_pitot(DMVector{state, control_dummy, dyn_params})[0].nonzeros()[0];

    /* Get aero values (Va, alpha, beta) */
    Va = m_NumericVa(DMVector{state, control_dummy, dyn_params})[0].nonzeros()[0];
    alpha = m_NumericAlpha(DMVector{state, control_dummy, dyn_params})[0].nonzeros()[0];
    beta = m_NumericBeta(DMVector{state, control_dummy, dyn_params})[0].nonzeros()[0];

    /* Get specific nongravitational force before solving (altering) the state */
    DM specNongravForce_evaluated = m_NumericSpecNongravForce(DMVector{state, control_dummy, dyn_params})[0];
    std::vector<double> specNongravForce_evaluated_vect = specNongravForce_evaluated.nonzeros();
    specNongravForce = specNongravForce_evaluated_vect;

    if (sim_tether)
    {
        /* Get tether force vector */
        DM specTethForce_evaluated = m_NumericSpecTethForce(DMVector{state, control_cmds, dyn_params})[0];
        std::vector<double> specTethForce_evaluated_vect = specTethForce_evaluated.nonzeros();
        specTethForce = specTethForce_evaluated_vect;
    }

    /* Get specific nongravitational force before solving (altering) the state */
    DM debug_evaluated = m_NumericDebug(DMVector{state, control_dummy, dyn_params})[0];
    std::vector<double> debug_evaluated_vect = debug_evaluated.nonzeros();
    //    std::cout << "debug_evaluated_vect: \n" << debug_evaluated_vect << "\n";

    sim_time += sim_dt;

}

void Simulator::publish_state(const ros::Time &sim_time)
{
    /** State message */
    std::vector<double> state_vec = state.nonzeros();

    std::vector<double> quat_vec = state(casadi::Slice(9, 13)).nonzeros(); // state contains quaternion
//    casadi::DM euler = state(casadi::Slice(9, 12)); // state contains euler angles
//    casadi::DM q_bn = kite_model::euler2quat(euler);
//    casadi::DM q_nb = polymath::quat_inverse(q_bn);
//    std::vector<double> quat_vec = q_nb.nonzeros();

    msg_state.header.stamp = sim_time;

    msg_state.twist[0].linear.x = state_vec[0];
    msg_state.twist[0].linear.y = state_vec[1];
    msg_state.twist[0].linear.z = state_vec[2];

    msg_state.twist[0].angular.x = state_vec[3];
    msg_state.twist[0].angular.y = state_vec[4];
    msg_state.twist[0].angular.z = state_vec[5];

    msg_state.transforms[0].translation.x = state_vec[6];
    msg_state.transforms[0].translation.y = state_vec[7];
    msg_state.transforms[0].translation.z = state_vec[8];

    msg_state.transforms[0].rotation.w = quat_vec[0];
    msg_state.transforms[0].rotation.x = quat_vec[1];
    msg_state.transforms[0].rotation.y = quat_vec[2];
    msg_state.transforms[0].rotation.z = quat_vec[3];

    /* Using wrench 0 as for acceleration, as measured by IMU (spec nongravitational forces) */
    msg_state.wrench[0].force.x = specNongravForce[0];
    msg_state.wrench[0].force.y = specNongravForce[1];
    msg_state.wrench[0].force.z = specNongravForce[2];

    /* Using wrench 1 as for acceleration */
    msg_state.wrench[1].force.x = specTethForce[0];
    msg_state.wrench[1].force.y = specTethForce[1];
    msg_state.wrench[1].force.z = specTethForce[2];

    /* Aero values */
    msg_state.twist[1].linear.x = Va;
    msg_state.twist[1].linear.y = beta;
    msg_state.twist[1].linear.z = alpha;
    msg_state.twist[1].angular.x = Va_pitot;
    /* Wind velocity vector */
    msg_state.twist[1].angular.y = Vw_N;
    msg_state.twist[1].angular.z = Vw_E;

    state_pub.publish(msg_state);

    /** Controls message **/
    msg_control.header.stamp = sim_time;

//    msg_control.axes[0] = (std::abs(state_vec[13]) < 0.001 ? 0.0 : state_vec[13]); // Thrust
    msg_control.axes[0] = (std::abs(thrust) < 0.001 ? 0.0 : thrust); // Thrust
    msg_control.axes[1] = (std::abs(state_vec[14]) < 0.001 ? 0.0 : state_vec[14]); // Elevator
    msg_control.axes[2] = (std::abs(state_vec[15]) < 0.001 ? 0.0 : state_vec[15]);; // Rudder
    msg_control.axes[3] = (std::abs(state_vec[16]) < 0.001 ? 0.0 : state_vec[16]);; // Aileron

    control_pub.publish(msg_control);

    /** UNCOMMENT THIS FOR TESTING ONLY **/
    /** Pixhawk/Mavros Pose messsage **/
//    geometry_msgs::PoseStamped msg_pose{};
//    msg_pose.header.stamp = sim_time;
//
//    /* Position in ENU frame */
//    msg_pose.pose.position.x = state_vec[7]; // Y+
//    msg_pose.pose.position.y = state_vec[6]; // X+
//    msg_pose.pose.position.z = -state_vec[8]; // Z-
//
//    /* Rotation from Front-Left-Up body frame to ENU frame */
//    DM q_nb = state(casadi::Slice(9, 13));
//    DM q_enu_ned = DM::vertcat({0, -sqrt(2) / 2.0, -sqrt(2) / 2.0, 0});
//    DM q_b_flu = DM::vertcat({0, -1, 0, 0});
//
//    DM q_enu_flu = polymath::quat_multiply(q_enu_ned, polymath::quat_multiply(q_nb, q_b_flu));
//    std::vector<double> q_enu_flu_vec = q_enu_flu.nonzeros();
//
//    msg_pose.pose.orientation.w = q_enu_flu_vec[0];
//    msg_pose.pose.orientation.x = q_enu_flu_vec[1];
//    msg_pose.pose.orientation.y = q_enu_flu_vec[2];
//    msg_pose.pose.orientation.z = q_enu_flu_vec[3];
//
//    pose_px_pub.publish(msg_pose);
//
//    geometry_msgs::TwistStamped msg_twist{};
//    msg_twist.header.stamp = sim_time;
//    /* Linear and angular velocity in Front-Left-Up body frame */
//    msg_twist.twist.linear.x = state_vec[0];
//    msg_twist.twist.linear.y = -state_vec[1];
//    msg_twist.twist.linear.z = -state_vec[2];
//    msg_twist.twist.angular.x = state_vec[3];
//    msg_twist.twist.angular.y = -state_vec[4];
//    msg_twist.twist.angular.z = -state_vec[5];
//    twist_px_pub.publish(msg_twist);
//
//    msg_twist.header.stamp = sim_time;
//
//    DM v_body = state(casadi::Slice(0, 3));
//    DM v_ned = polymath::quat_transform(q_nb, v_body);
//    std::vector<double> v_ned_vec = v_ned.nonzeros();
//    /* Local velocity in ENU frame */
//    msg_twist.twist.linear.x = v_ned_vec[1];
//    msg_twist.twist.linear.y = v_ned_vec[0];
//    msg_twist.twist.linear.z = -v_ned_vec[2];
//    msg_twist.twist.angular.x = 0;
//    msg_twist.twist.angular.y = 0;
//    msg_twist.twist.angular.z = 0;
//    local_vel_px_pub.publish(msg_twist);
}

void Simulator::publish_pose(const ros::Time &sim_time)
{
    std::vector<double> pos_vec = state(casadi::Slice(6, 9)).nonzeros();

    std::vector<double> quat_vec = state(casadi::Slice(9, 13)).nonzeros(); // state contains quaternion
//    casadi::DM euler = state(casadi::Slice(9, 12)); // state contains euler angles
//    casadi::DM q_bn = kite_model::euler2quat(euler);
//    casadi::DM q_nb = polymath::quat_inverse(q_bn);
//    std::vector<double> quat_vec = q_nb.nonzeros();

    geometry_msgs::PoseStamped msg_pose;
    msg_pose.header.stamp = sim_time;

    msg_pose.pose.position.x = pos_vec[0];
    msg_pose.pose.position.y = pos_vec[1];
    msg_pose.pose.position.z = pos_vec[2];

    msg_pose.pose.orientation.w = quat_vec[0];
    msg_pose.pose.orientation.x = quat_vec[1];
    msg_pose.pose.orientation.y = quat_vec[2];
    msg_pose.pose.orientation.z = quat_vec[3];

    pose_pub.publish(msg_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle n("~");

    /** create a kite object */
    std::string kite_params_file;
    n.param<std::string>("kiteparams", kite_params_file, "kiteparams");
    if (kite_params_file.at(0) == '/'); /* Full path given, don't modify */
    else /* Add path to kite_model files */
        kite_params_file = ros::package::getPath("kite_model") + "/config/" + kite_params_file;
    if (kite_params_file.find(".yaml") == std::string::npos) /* Add .yaml ending */
        kite_params_file.append(".yaml");

    std::cout << "Simulator: Using kite parameters from: " << kite_params_file << "\n";

    double windFrom_deg{180};
    double windSpeed{0};
    n.param<double>("windFrom_deg", windFrom_deg, 180.0);
    n.param<double>("windSpeed", windSpeed, 0.0);
    const double airDensity = 1.1589; // Standard atmosphere at 468 meters

    bool simulate_turbulence{false};
    n.param<bool>("simulate_turbulence", simulate_turbulence, true);

    bool simulate_tether;
    n.param<bool>("simulate_tether", simulate_tether, false);
    if (simulate_tether)
        std::cout << "[Simulator]: Tether ON\n";

    /** Setup static, dynamic, and optimizable parameters ---------------------------------------------------------- **/
    /* Static parameters: Set {name, value} pair once that will be constant afterwards */
    const std::map<std::string, double> staticParams = {{"air_density", airDensity}};//,
    //{"v_wind_N",    vWind_N},
    //{"v_wind_E",    vWind_E}};

    /* Dynamic parameters: Set parameter names that will be modifiable before running an optimization  */
    const std::vector<std::string> dynParamNames = {"v_wind_N", "v_wind_E"};

    /** Kite Dynamics ---------------------------------------------------------------------------------------------- **/
    /* Construct kite dynamics for forward integration (for initial guess) */
    kite_model::KiteDynamics kiteDynamics = kite_model::AugKiteDynamics(kite_params_file, kite_model::AttQuat,
                                                                        staticParams, dynParamNames, {},
                                                                        simulate_tether);
    Function ode = kiteDynamics.getNumericDynamics(true);

    int broadcast_state;
    n.param<int>("broadcast_state", broadcast_state, 1);

    /** create an integrator instance */
    double node_rate;
    n.param<double>("node_rate", node_rate, 200);

    double sim_speed;
    n.param<double>("simulation_speed", sim_speed, 1.0);

    /** cast to seconds and round to ms */
    double dt = (1.0 / node_rate);
    dt = std::roundf(sim_speed * dt * 1000) / 1000;

    Dict params({{"tf",     dt},
                 {"tol",    1e-6},
                 {"method", CVODES}});
    ODESolver odeSolver(ode, params);

    Simulator simulator(odeSolver, n);
    simulator.sim_tether = simulate_tether;
    simulator.setNumericThrust(kiteDynamics.getNumericOutput("thrust", true));
    simulator.setNumericVa(kiteDynamics.getNumericOutput("Va", true));
    simulator.setNumericAlpha(kiteDynamics.getNumericOutput("alpha", true));
    simulator.setNumericBeta(kiteDynamics.getNumericOutput("beta", true));
    simulator.setNumericVaPitot(kiteDynamics.getNumericOutput("Va_pitot", true));
    simulator.setNumericSpecNongravForce(kiteDynamics.getNumericOutput("spec_nongrav_force", true));
    simulator.setNumericSpecTethForce(kiteDynamics.getNumericOutput("spec_tether_force", true));
    simulator.setNumericDebug(kiteDynamics.getNumericOutput("debugSX", true));
    simulator.sim_dt = dt;

    simulator.Vw_N = windSpeed * -cos(windFrom_deg * M_PI / 180.0);
    simulator.Vw_E = windSpeed * -sin(windFrom_deg * M_PI / 180.0);
    if (simulate_turbulence)
    {
        std::cout << "Simulator: Wind from " << windFrom_deg << " deg at " << windSpeed << " m/s (turbulence ON).\n";
        simulator.sim_turbulence = true;
        simulator.discreteGustGenerator.init(windSpeed, windFrom_deg * M_PI / 180.0, 2, 0.2 * M_PI / 180);
//        simulator.drydenWind.initialize(simulator.Vw_N, simulator.Vw_E, 0, 0.5 * windSpeed, 0.5 * windSpeed, 0);
    }
    else
    {
        std::cout << "Simulator: Wind from " << windFrom_deg << " deg at " << windSpeed << " m/s (turbulence OFF).\n";
    }

    ros::Rate loop_rate(node_rate);

    while (ros::ok())
    {
        if (simulator.is_initialized())
        {
            simulator.simulate();

//            double sim_time_sec = (ros::Time::now() - simulation_time_start).toSec() * sim_speed;
            ros::Time sim_time;
//            sim_time.fromSec(sim_time_sec);
            sim_time = ros::Time::now();

            if (broadcast_state)
                simulator.publish_state(sim_time);
            else
                simulator.publish_pose(sim_time);

            if (-simulator.getState().nonzeros()[8] < 0)
            {
                std::cout << "\n----------------------------------\n"
                             "Hit the ground. End of simulation.\n"
                          << "----------------------------------\n";
                return 0;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
