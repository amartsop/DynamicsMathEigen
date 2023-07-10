#pragma once 
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace dme
{
    // Cartesian structure 
    struct Cartesian {
        double time = 0.0;
        Eigen::Vector3d rop_F_F = {0.0, 0.0, 0.0};
        Eigen::Vector3d rop_F_F_dot = {0.0, 0.0, 0.0};
        Eigen::Vector3d rop_F_F_ddot = {0.0, 0.0, 0.0};
        Eigen::Vector3d euler = {0.0, 0.0, 0.0};
        Eigen::Vector3d w_f_F_F = {0.0, 0.0, 0.0};
        Eigen::Vector3d w_f_F_F_dot = {0.0, 0.0, 0.0};
    };

    // Joint space path struct
    struct Joint {
        double time;
        Eigen::VectorXd q;
        Eigen::VectorXd q_dot;
        Eigen::VectorXd q_ddot;
    };

    // General state representation
    struct State {
        Eigen::VectorXd q;
        Eigen::VectorXd q_dot;
        Eigen::VectorXd q_ddot;
    };

    // Joint space path struct
    struct FTData {
        double time;
        Eigen::Vector3d forces = {0.0, 0.0, 0.0};
        Eigen::Vector3d moments = {0.0, 0.0, 0.0};
    };

    // Skew-symmetric matrix 
    Eigen::Matrix3d S(const Eigen::Vector3d& r);
     
    /**
     * @brief This function calculates the cartesian state of a frame fp with
     * respect to an inertial frame F, given the cartesian state of a frame 
     * fe. For this calculation we assume that there is no relative motion between 
     * the fames fp and fe. The function also requires the rotation matrix 
     * of the frame fp with respect fe (rot_fp_fe) and the relative distance of 
     * the two frames expressed in the frame fe (rep_fe_fe).
     * @param fe_state The cartesian state of the frame fe.
     * @param rot_fp_fe The rotation matrix of the frame fp with respect fe
     * (rot_fp_fe).
     * @param rep_fe_fe The relative distance of the two frames expressed in
     * the frame fe.
     * @return dme::Cartesian The cartesian staste of the frame fp
     */
    dme::Cartesian state_transformation(const dme::Cartesian& fe_state,
        const Eigen::Matrix3d& rot_fp_fe, const Eigen::Vector3d& rep_fe_fe);

    // Euler rotations class
    class EulerRotations
    {
    public:

        // Eulear angles struct
        struct Euler{ double phi, theta, psi; };

        /** Euler angles to quaternions. Euler angles follow the post multiply
            sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
            and "phi" around x (roll) **/
        static Eigen::Quaterniond euler_to_quaternions(double phi, double
            theta, double psi);

        /** Quaternions to Euler Angles. Euler angles follow the post multiply
            sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
            and "phi" around x (roll) **/
        static Eigen::Vector3d quaternions_to_euler(const Eigen::Quaterniond& q);

    public:
        // Basic rotation matrix wrt x axis
        static Eigen::Matrix3d basic_rotation_x(double x);

        // Basic rotation matrix wrt y axis
        static Eigen::Matrix3d basic_rotation_y(double x);

        // Basic rotation matrix wrt z axis
        static Eigen::Matrix3d basic_rotation_z(double x);

        // Euler rotation matrix z-y'-x''
        static Eigen::Matrix3d rotation(double phi, double theta, double psi);
        static Eigen::Matrix3d rotation(Eigen::Vector3d euler_angles);
        static Eigen::Matrix3d rotation(std::vector<double> euler_angles);
        static Eigen::Matrix3d rotation(Euler euler_angles);

        // Euler angles derivative to body anglular velocity mapping (w = G * theta_dot)
        static Eigen::Matrix3d G(double phi, double theta, double psi);
        static Eigen::Matrix3d G(Eigen::Vector3d euler_angles);
        static Eigen::Matrix3d G(std::vector<double> euler_angles);
        static Eigen::Matrix3d G(Euler euler_angles);

        // Euler angles second derivative to body anglular acceleration mapping
        static Eigen::Matrix3d G_dot(Eigen::Vector3d euler_angles,
            Eigen::Vector3d euler_angles_dot);

        static Eigen::Matrix3d G_dot(std::vector<double> euler_angles,
            std::vector<double> euler_angles_dot);
    
        // Rotation matrix to euler angles
        static Eigen::Vector3d rotation_to_euler(const Eigen::Matrix3d& rot_mat);
    };
}