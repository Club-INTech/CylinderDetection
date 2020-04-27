//
// Created by serandour on 24/02/2020.
//

#include <librealsense2/rs.hpp>
#include <mutex>
#include <cstring>
#include <cmath>
#include <Eigen/Dense>
#include "CameraLocation.h"


#ifndef PI
const double PI = 3.14159265358979323846;
#endif

class rotation_estimator
{
    // theta is the angle of camera rotation in x, y and z components
    float3 theta{0.0f,0.0f, 0.0f};
    std::mutex theta_mtx;
    float3 position{};
    float3 speed{};
    std::chrono::time_point<std::chrono::system_clock> start;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98;
    bool first = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
public:
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts)
    {
        if (first) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = (ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle.x = gyro_angle.x * dt_gyro;
        gyro_angle.y = gyro_angle.y * dt_gyro;
        gyro_angle.z = gyro_angle.z * dt_gyro;

        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.x-=gyro_angle.z;
        theta.y-=gyro_angle.y;
        theta.z+=gyro_angle.x;
    }

    void process_accel(rs2_vector accel_data, double ts)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;
        float3 accel_pos;

        accel_pos.x = accel_data.x;
        accel_pos.y = accel_data.y;
        accel_pos.z = accel_data.z;

        if(!first) {
            calc_position(accel_pos);
        }

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (first)
        {
            last_ts_gyro = ts;
            first = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = PI;
        }
        else
        {
            /*
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }

    // Returns the current rotation angle
    float3 get_theta(){
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }

    float3 get_position(){
        return position;
    }

    void set_position(float3 pos){
        position = pos;
    }

    void set_clock(std::chrono::time_point<std::chrono::system_clock> debut){
        this->start = debut;
    }

    std::chrono::time_point<std::chrono::system_clock> get_clock(){
        return this->start;
    }

    float3 get_speed(){
        return speed;
    }

    void set_speed(float3 vitesse) {
        speed = vitesse;
    }
};

static rotation_estimator rotate;

void initRotationEstimator() {
    rotate.set_clock(std::chrono::system_clock::now());
}

void estimateCameraPositionRotation(rs2::frame& frame) {
    // Cast the frame that arrived to motion frame
    auto motion = frame.as<rs2::motion_frame>();
    // If casting succeeded and the arrived frame is from gyro stream
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        // Get the timestamp of the current frame
        double ts = motion.get_timestamp();
        // Get gyro measures
        rs2_vector gyro_data = motion.get_motion_data();
        // Call function that computes the angle of motion based on the retrieved measures
        rotate.process_gyro(gyro_data, ts);
    }
    // If casting succeeded and the arrived frame is from accelerometer stream
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        double ts = motion.get_timestamp();
        // Get accelerometer measures
        rs2_vector accel_data = motion.get_motion_data();
        // Call function that computes the angle of motion based on the retrieved measures
        rotate.process_accel(accel_data, ts);
    }
}

void get_rotation(float3* out) {
    float3 theta = rotate.get_theta();
    out->x = theta.x;
    out->y = theta.y;
    out->z = theta.z;
}

void get_position(float3* out) {
    float3 position = rotate.get_position();
    out->x = position.x;
    out->y = position.y;
    out->z = position.z;
}

void calc_position(float3 accel_pos){
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-rotate.get_clock();
    float dt = elapsed_seconds.count();

    // basé sur https://github.com/IntelRealSense/librealsense/blob/master/examples/motion/rs-motion.cpp#L84
    Eigen::AngleAxisf pitchAngle((rotate.get_theta().z-PI/2), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngle(-rotate.get_theta().y, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(rotate.get_theta().x, Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
    q = q.conjugate();

    Eigen::Matrix3f rotationMatrix = q.matrix();
    Eigen::Vector3f acceleration(accel_pos.x, accel_pos.y, accel_pos.z);
    Eigen::Vector3f accel_correct = rotationMatrix*acceleration;

    float gravity_average = 9.722579856f; // basé sur 126 mesures de suite avec la caméra posée au sol
    float3 accel_corrected{accel_correct.x(), accel_correct.y()-gravity_average, accel_correct.z()};
    rotate.set_speed(rotate.get_speed() + accel_corrected*dt);
    rotate.set_position(rotate.get_position() + rotate.get_speed() * dt);
    printf("%f, %f, %f\n", accel_corrected.x, accel_corrected.y, accel_corrected.z);
    rotate.set_clock(end);
}
