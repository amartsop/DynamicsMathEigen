#include "../include/dynamics_math_eigen.h"

// Skew symmetric matrix
Eigen::Matrix3d dme::S(const Eigen::Vector3d& r)
{
    Eigen::Matrix3d s_mat;
    s_mat << 0.0, -r(2), r(1), r(2), 0.0, -r(0), -r(1), r(0), 0.0;
        
    return s_mat;
}

// Basic rotation matrix wrt x axis
Eigen::Matrix3d dme::EulerRotations::basic_rotation_x(double x)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << 1.0f, 0.0f, 0.0f, 
        0.0f, cos(x), -sin(x), 
        0.0f, sin(x), cos(x);
    return m;
}

// Basic rotation matrix wrt y axis
Eigen::Matrix3d dme::EulerRotations::basic_rotation_y(double x)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << cos(x), 0.0f, sin(x), 
        0.0f, 1.0f, 0.0f, 
        -sin(x), 0.0f, cos(x);
    return m;
}


// Basic rotation matrix wrt z axis
Eigen::Matrix3d dme::EulerRotations::basic_rotation_z(double x)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << cos(x), -sin(x), 0.0f, 
        sin(x), cos(x), 0.0f, 
        0.0f, 0.0f, 1.0f;
    return m;
}


// Euler rotation matrix z-y'-x''
Eigen::Matrix3d dme::EulerRotations::rotation(double phi, double theta, double psi)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    // Basic rotations
    Eigen::Matrix3d rotx =  basic_rotation_x(phi);
    Eigen::Matrix3d roty = basic_rotation_y(theta);
    Eigen::Matrix3d rotz = basic_rotation_z(psi);

    // Total rotation matrix
    m = rotz * roty * rotx;

    return m;
}


// Euler rotation matrix z-y'-x''
Eigen::Matrix3d dme::EulerRotations::rotation(Eigen::Vector3d euler_angles)
{
    return rotation(euler_angles(0), euler_angles(1), euler_angles(2));
}


// Euler rotation matrix z-y'-x''
Eigen::Matrix3d dme::EulerRotations::rotation(std::vector<double> euler_angles)
{
    return rotation(euler_angles[0], euler_angles[1], euler_angles[2]);
}

// Euler rotation matrix z-y'-x''
Eigen::Matrix3d dme::EulerRotations::rotation(Euler euler_angles)
{
    return rotation(euler_angles.phi, euler_angles.theta, euler_angles.psi);
}

// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d dme::EulerRotations::G(double phi, double theta, double psi)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << 1.0, 0.0, -sin(theta), 
        0.0, cos(phi), cos(theta) * sin(phi), 
        0.0, -sin(phi), cos(phi) * cos(theta);

    return m;
}


// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d dme::EulerRotations::G(Eigen::Vector3d euler_angles)
{
    return G(euler_angles(0), euler_angles(1), euler_angles(2));
}


// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d dme::EulerRotations::G(std::vector<double> euler_angles)
{
    return G(euler_angles[0], euler_angles[1], euler_angles[2]);
}


// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d dme::EulerRotations::G(Euler euler_angles)
{
    return G(euler_angles.phi, euler_angles.theta, euler_angles.psi);
}


// Euler angles second derivative to body anglular acceleration mapping
Eigen::Matrix3d dme::EulerRotations::G_dot(Eigen::Vector3d euler_angles,
    Eigen::Vector3d euler_angles_dot)
{
    double phi = euler_angles(0);
    double theta = euler_angles(1);

    double phi_dot = euler_angles_dot(0);
    double theta_dot = euler_angles_dot(1);

    // Matrix initialization
    Eigen::Matrix3d m;

    m << 0.0, 0.0, - cos(theta) * theta_dot,
        0.0, - sin(phi) * phi_dot, cos(theta) * cos(phi) * phi_dot - 
        sin(theta) * sin(phi) * theta_dot,
        0.0, - cos(phi) * phi_dot, -sin(phi) * cos(theta) * phi_dot - 
        cos(phi) * sin(theta) * theta_dot;

    return m;
}


// Euler angles second derivative to body anglular acceleration mapping
Eigen::Matrix3d dme::EulerRotations::G_dot(std::vector<double> euler_angles,
        std::vector<double> euler_angles_dot)
{
    double phi = euler_angles[0];
    double theta = euler_angles[1];

    double phi_dot = euler_angles_dot[0];
    double theta_dot = euler_angles_dot[1];

    // Matrix initialization
    Eigen::Matrix3d m;

    m << 0.0, 0.0, - cos(theta) * theta_dot,
        0.0, - sin(phi) * phi_dot, cos(theta) * cos(phi) * phi_dot - 
        sin(theta) * sin(phi) * theta_dot,
        0.0, - cos(phi) * phi_dot, -sin(phi) * cos(theta) * phi_dot - 
        cos(phi) * sin(theta) * theta_dot;

    return m;
}

/** Euler angles to quaternions. Euler angles follow the post multiply
    sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
    and "phi" around x (roll) **/
Eigen::Quaterniond dme::EulerRotations::euler_to_quaternions(
    double phi, double theta, double psi)
{
    Eigen::Quaterniond quatern;

    double cy = cos(psi * 0.5); double sy = sin(psi * 0.5);
    double cp = cos(theta * 0.5); double sp = sin(theta * 0.5);
    double cr = cos(phi * 0.5); double sr = sin(phi * 0.5);

    quatern.w() = cr * cp * cy + sr * sp * sy;
    quatern.x() = sr * cp * cy - cr * sp * sy;
    quatern.y() = cr * sp * cy + sr * cp * sy;
    quatern.z() = cr * cp * sy - sr * sp * cy;

    // Return Quaternions
    return quatern;
}


/** Quaternions to Euler Angles. Euler angles follow the post multiply
    sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
    and "phi" around x (roll) **/
Eigen::Vector3d dme::EulerRotations::quaternions_to_euler(const
    Eigen::Quaterniond& q)
{
    Eigen::Vector3d euler_angles; euler_angles.setZero();

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    euler_angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1.0)
    {
        euler_angles(1) = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
    }
    else
    {
        euler_angles(1) = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    euler_angles(2) = std::atan2(siny_cosp, cosy_cosp);

    // Return Euler angles
    return euler_angles;
}

// Rotation matrix to euler angles
Eigen::Vector3d dme::EulerRotations::rotation_to_euler(const Eigen::Matrix3d& rot_mat)
{
    // Rotation matrix components
    double r11 = rot_mat(0, 0);
    double r12 = rot_mat(0, 1);
    double r13 = rot_mat(0, 2);

    double r21 = rot_mat(1, 0);

    double r31 = rot_mat(2, 0);
    double r32 = rot_mat(2, 1);
    double r33 = rot_mat(2, 2);

    // Initialize euler angles
    double phi, theta, psi;

    if (r31 == 1 || r31 == -1) {
        // Set psi aribtrarily
        psi = 0;

        if(r31 == - 1) { theta = M_PI_2; phi = psi + atan2(r12, r13); }
        else {theta = - M_PI_2, phi = -psi + atan2(-r12, -r13); }
    }
    else {
        theta = - asin(r31);
        phi = atan2( (r32 / cos(theta)), (r33 / cos(theta)) );
        psi = atan2( (r21 / cos(theta)), (r11 / cos(theta)) );
    }

    return {phi, theta, psi};
}


dme::Cartesian dme::state_transformation(const dme::Cartesian& fe_state,
    const Eigen::Matrix3d& rot_fp_fe, const Eigen::Vector3d& rep_fe_fe)
{
    // Initialize the state of the fp frame
    dme::Cartesian fp_state;

    // Get the rotation matrix of the fe frame with respect to the F frame
    Eigen::Matrix3d rot_fe_F = dme::EulerRotations::rotation(fe_state.euler);

    // Get the position of the fp frame with respect to the F frame 
    fp_state.rop_F_F = fe_state.rop_F_F + rot_fe_F * rep_fe_fe;

    // Get the velocity of the fp frame with respect to the F frame 
    fp_state.rop_F_F_dot = fe_state.rop_F_F_dot +
        dme::S(fe_state.w_f_F_F) * rot_fe_F * rep_fe_fe;
    
    // Get the acceleration of the fp frame with respect to the F frame 
    fp_state.rop_F_F_ddot = fe_state.rop_F_F_ddot +
        (dme::S(fe_state.w_f_F_F_dot) + dme::S(fe_state.w_f_F_F) *
        dme::S(fe_state.w_f_F_F) ) * rot_fe_F * rep_fe_fe;

    // Get the euler angles of the fp frame with respect to the F frame        
    Eigen::Matrix3d rot_fp_F = rot_fe_F * rot_fp_fe;
    Eigen::Quaterniond q_fp_F(rot_fp_F);
    fp_state.euler = dme::EulerRotations::quaternions_to_euler(q_fp_F);
    
    // Get rotational velocity for the fp frame with respect to the F frame 
    fp_state.w_f_F_F = fe_state.w_f_F_F;
    
    // Get rotational acceleration for the fp frame with respect to the F frame 
    fp_state.w_f_F_F_dot = fe_state.w_f_F_F_dot;

    return fp_state;
}