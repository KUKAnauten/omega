#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "omega/drdc.h" // includes dhdc.h

// Calculating quaternion from euler angles (moving frame). Rotation order is xyz.
void eulerXYZToQuaternion(const tf2Scalar& yaw, const tf2Scalar& pitch, const tf2Scalar& roll, tf2::Quaternion& q)
{
       tf2Scalar halfYaw = tf2Scalar(yaw) * tf2Scalar(0.5);
       tf2Scalar halfPitch = tf2Scalar(pitch) * tf2Scalar(0.5);
       tf2Scalar halfRoll = tf2Scalar(roll) * tf2Scalar(0.5);
       tf2Scalar cosYaw = tf2Cos(halfYaw);
       tf2Scalar sinYaw = tf2Sin(halfYaw);
       tf2Scalar cosPitch = tf2Cos(halfPitch);
       tf2Scalar sinPitch = tf2Sin(halfPitch);
       tf2Scalar cosRoll = tf2Cos(halfRoll);
       tf2Scalar sinRoll = tf2Sin(halfRoll);
       q.setValue(cosRoll * cosPitch * sinYaw + sinRoll * sinPitch * cosYaw,
               cosRoll * sinPitch * cosYaw - sinRoll * cosPitch * sinYaw,
               sinRoll * cosPitch * cosYaw + cosRoll * sinPitch * sinYaw,
               cosRoll * cosPitch * cosYaw - sinRoll * sinPitch * sinYaw);
}

// Enables/disables regulation for position, orientation and gripper. Starts regulation thread.
// Disabled: Part moves freely and displays forces set using drdSetForceAndTorqueAndGripperForce()
// Enabled: Part is locked and can be controlled by calling robotic functions (e.g. drdMoveToGrip())
bool startRegulationThread(bool regPos, bool regRot, bool regGrip)
{
  drdRegulatePos(regPos);
  drdRegulateRot(regRot);
  drdRegulateGrip(regGrip);
  drdStart(); // Starts regulation thread. Also enables forces.
  return drdIsRunning();
}

// PoseStamped used for the 7 DOF of Omega. Should be replaced with proper message format.
void forceCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{  
  double fx_desired = msg->pose.position.x;
  double fy_desired = msg->pose.position.y;
  double fz_desired = msg->pose.position.z;
  double tx_desired = msg->pose.orientation.x; 
  double ty_desired = msg->pose.orientation.y;
  double tz_desired = msg->pose.orientation.z;
  double fgrip_desired = msg->pose.orientation.w;

  drdSetForceAndTorqueAndGripperForce(fx_desired, fy_desired, fz_desired, tx_desired, ty_desired, tz_desired, fgrip_desired); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "omega");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Publisher
  ros::Publisher omega_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("omega_out_pose", 1);
  ros::Publisher omega_grip_angle_pub = nh.advertise<std_msgs::Float64>("omega_out_grip_angle", 1);
  ros::Publisher omega_grip_gap_pub = nh.advertise<std_msgs::Float64>("omega_out_grip_gap", 1);

  ros::Publisher omega_deg_pub = nh.advertise<geometry_msgs::PoseStamped>("omega_out_deg", 1);

  ros::Publisher omega_force_pub = nh.advertise<geometry_msgs::PoseStamped>("omega_out_force", 1);

  // Subscriber
  ros::Subscriber omega_force_sub = nh.subscribe("omega_in_force", 1, forceCallback);

  // Messages
  geometry_msgs::PoseStamped omega_pose;
  tf2::Quaternion omega_quat;
  std_msgs::Float64 omega_grip_angle;
  std_msgs::Float64 omega_grip_gap;
  geometry_msgs::PoseStamped omega_force; // PoseStamped used for the 7 DOF of Omega. Should be replaced with proper message format.

  double px, py, pz;
  double ox_rad, oy_rad, oz_rad;
  double grip_angle_rad, grip_gap;

  double fx, fy, fz;
  double tx, ty, tz;
  double f_grip;

  double t0, t1, delta_t, freq;

  int enc_grip;
  double gap_grip;

  int success;

  // open the first available device
  if (drdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  // identify device
  printf ("%s device detected\n\n", dhdGetSystemName());

  startRegulationThread(false, false, false); // Position regulation off, orientation regulation off, gripper regulation off.

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::Time stamp = ros::Time::now();
    dhdGetPositionAndOrientationRad(&px, &py, &pz, &ox_rad, &oy_rad, &oz_rad);
    dhdGetGripperAngleRad(&grip_angle_rad);
    dhdGetGripperGap(&grip_gap);
    dhdGetForceAndTorqueAndGripperForce(&fx, &fy, &fz, &tx, &ty, &tz, &f_grip);

    eulerXYZToQuaternion(tf2Scalar(ox_rad), tf2Scalar(oy_rad), tf2Scalar(oz_rad), omega_quat);

    omega_pose.header.stamp = stamp;
    omega_pose.pose.position.x = px;
    omega_pose.pose.position.y = py;
    omega_pose.pose.position.z = pz;
    omega_pose.pose.orientation.x = omega_quat.getX();
    omega_pose.pose.orientation.y = omega_quat.getY();
    omega_pose.pose.orientation.z = omega_quat.getZ();
    omega_pose.pose.orientation.w = omega_quat.getW();

    omega_grip_angle.data = grip_angle_rad;
    omega_grip_gap.data = grip_gap;

    omega_force.header.stamp = stamp;
    omega_force.pose.position.x = fx;
    omega_force.pose.position.y = fy;
    omega_force.pose.position.z = fz;
    omega_force.pose.orientation.x = tx;
    omega_force.pose.orientation.y = ty;
    omega_force.pose.orientation.z = tz;
    omega_force.pose.orientation.w = f_grip;

    // Publish omega pose
    omega_pose_pub.publish(omega_pose);
    omega_grip_angle_pub.publish(omega_grip_angle);
    omega_grip_gap_pub.publish(omega_grip_gap);

    // Publish omega force
    omega_force_pub.publish(omega_force);


    loop_rate.sleep();
  }

  drdStop();

  drdClose();
  printf("\ndone\n");

}
