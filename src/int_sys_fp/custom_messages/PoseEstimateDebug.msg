std_msgs/Header header
uint8 robot_id

# Posterior state estimate (plain doubles, not quaternion-encoded - easy Matlab import)
float64 x
float64 y
float64 theta

# Posterior covariance P, 3x3 row-major flattened: [Pxx,Pxy,Pxth, Pyx,Pyy,Pyth, Pthx,Pthy,Pthth]
float64[9] p

# Measurement vector layout for z / z_pred / innovation (6 elements):
# [anchor0_dist, anchor1_dist, anchor2_dist, neighbor0_dist, neighbor1_dist, theta_imu]
float64[6] z              # raw measurement actually used this cycle
float64[6] z_pred         # h(x_pred) as computed internally by the filter (sigma-point mean for UKF)
float64[6] innovation     # z - z_pred (theta component angle-wrapped)

float64[3] q_diag         # process noise diagonal used this cycle (includes R_omega*dt^2 inflation on theta)
float64[6] r_diag         # measurement noise diagonal used this cycle

bool have_imu             # whether an IMU message had been received before this update
float64 v_cmd             # linear velocity input used (from cmd_vel)
float64 omega_imu         # angular velocity input used (from IMU gyro z)
float64 dt
