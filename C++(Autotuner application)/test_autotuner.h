
/* This application is written to test
 * pso optimisation on single leg
 *
 * This application is run in the host system which
 * sends command to the Raspberry Pi. The RPi further
 * forwards the commands to the motors.
 *
 * This application commands two motors with IDs 1,2
 * with position commands to track trajectories.
 * It sends the commands and subsequently requests for the
 * response on the motor status. It also updates KP, KD on the go based on the PSO algorithm
 *
 * See also: main.cpp
 * Author: Prakhar Goel
 */

#ifndef __TEST_AUTOTUNER__
#define __TEST_AUTOTUNER__

#include <fstream>

#include "kinematics/serial2r_kinematics.h"
#include "kinematics/utils/transformations.h"
#include <stdio.h>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <vector>

#include "eth_comm.h"
#include "pi3hat/pi3hat_interface.h"
#include "serialize.h"


using namespace kine;

#define PEER_IP "169.254.216.146"
#define PEER_PORT 8888
#define SELF_PORT 9999

// Control time-period in milliseconds
#define PERIOD_MS 5
#define BUFFER_SIZE 2048

typedef enum
{
  eCmdMotorInit = 0x00,
  eCmdMotorStop = 0x01,
  eCmdMotorCmd = 0x02,
  eCmdMotorState = 0x03,
  eCmdIMUState = 0x04
} eCmd;

// Autotuner params
#define MAX_GENERATIONS 20
#define MAX_AGENTS 10

namespace autotuner_pd
{
class autotuner
{
private:
  // trajectory params and variables
  double s_l_;
  double w_h_;
  double z_foot_;
  double x_, y_, v_x_, v_y_;
  double theta;
  double omega;

  Serial2RKinematics serial_2r_;
  utils::Vector2d joint_angles_;
  utils::Vector2d joint_velocities_;
  utils::Vector2d ee_pos_;
  utils::Vector2d ee_vel_;
  utils::Matrix2d jac_inverse_;

  // PSO params
  double c1_swarm_;
  double c2_swarm_;
  double gamma_1_swarm_;
  double gamma_2_swarm_;
  double omega_swarm_;
  double omega_swarm_max_;
  double omega_swarm_min_;
  double max_velocity_gain_;
  utils::Matrix<double, 4, MAX_AGENTS> pos_swarm_;  //(kp1,kd1,kp2,kd2) =4x1
  utils::Matrix<double, 4, MAX_AGENTS> p_best_swarm_;
  utils::Vector<double, 4> g_best_swarm_;
  utils::Vector<double, MAX_AGENTS> iae_swarm_global_;

  struct oscillation_params {

    utils::VectorX<double> error_rescaled_link_1_;
    utils::VectorX<double> error_rescaled_link_2_ ;
    double index_ = 0;
    double index_max_ = 0.4;
    double nzc_ = 0;
    double n_ha_ = 0;
    double n_hb_ = 0;
    utils::VectorX<double> detla_tzc_link_1_;
    utils::VectorX<double> detla_tzc_link_2_;
    utils::VectorX<double> A_link_1_;
    utils::VectorX<double> A_link_2_;
    utils::VectorX<double> B_link_1_;
    utils::VectorX<double> B_link_2_;
    utils::VectorX<double> epsilon_link_1_;
    utils::VectorX<double> epsilon_link_2_;
    utils::VectorX<double> delta_link_1_;
    utils::VectorX<double> delta_link_2_;
    double alpha_;
    double gamma_;


  } oscillation_;

  public : autotuner(void) {
    s_l_ = 0.16;
    w_h_ = 0.15;
    z_foot_ = -0.44;
    x_ = 0, y_ = 0, v_x_ = 0, v_y_ = 0;
    theta = 0;
    omega = 2 * 1 * M_PI;  // 1 hz frequency

    c1_swarm_ = 1.0;
    c2_swarm_ = 1.0;
    gamma_1_swarm_ = 0.2;
    gamma_2_swarm_ = 0.2;
    omega_swarm_ = 0;
    omega_swarm_max_ = 0.9;
    omega_swarm_min_ = 0.4;
    max_velocity_gain_ = 0.2;  // 20%

    std::fstream fout;
    fout.open("/home/pi/Stoch3_hardware_interface/stoch3_hardware_interface/test/log_autotuner_().csv",
              std::ios::out | std::ios::app);
  }

  int state_ = 0;        // fsm state
  int next_agent_ = 0;   // fsm input
  int calc_error_ = 0;   // fsm process
  int hardware_ = 0;     // To use hardware interaction
  int test_cycles_ = 0;  // no_of seconds to interact with hardware
  int reset_ = 0;        // To activate reset in hardware interface
  double kp_1_ = 0, kp_2_ = 0, kd_1_ = 0, kd_2_ = 0;  // best manual tune start-point
  double kp_min_ = 1;
  double kp_max_ = 100.0;
  double kd_min_ = 0;
  double kd_max_ = 50;
  double search_init_low_[5] = { kp_1_ - kp_1_ / 2, kd_1_ - kd_1_ / 2, kp_2_ - kp_2_ / 2, kd_2_ - kd_2_ / 2 };
  double search_init_high_[5] = { kp_1_ + kp_1_ / 2, kp_1_ - kp_1_ / 2, kp_2_ - kp_2_ / 2, kd_2_ - kd_2_ / 2 };
  double e_max_ = M_PI / 10;
  double iae_swarm_local_ = 0;
  double iae_swarm_local_prev_ = 0;
  double error_joint_0 = 0;
  double error_joint_1 = 0;
  utils::Matrix<double, 4, MAX_AGENTS> vel_swarm_;  // defined public as main calls this when constraints check is true

  const double dt = 0.001 * PERIOD_MS;
  const int OSCILLATION_CYCLES_ = (int)(0.2 * test_cycles_ /dt);
  int oscillation_counter_ = 0;



  void reset_fsm_(void)
  {
    // after each agent moves
    state_ = 0;
    next_agent_ = 0;
    calc_error_ = 0;
    hardware_ = 0;
    iae_swarm_local_prev_ = iae_swarm_local_;
    iae_swarm_local_ = 0;

  }

  void init_fsm_(void)
  {
    for (int k_ = 0; k_ < 4; k_++)
    {
      pos_swarm_.row(k_) = pos_swarm_.row(k_).setLinSpaced(MAX_AGENTS, search_init_low_[k_], search_init_high_[k_]);
    }
    vel_swarm_ = vel_swarm_.Zero();
    p_best_swarm_ = pos_swarm_;
    g_best_swarm_ << kp_1_, kd_1_, kp_2_, kd_2_;
    iae_swarm_global_ = iae_swarm_global_.Zero();
    state_ = 0;
    next_agent_ = 0;
    calc_error_ = 0;
    hardware_ = 0;
    iae_swarm_local_ = 0;
    iae_swarm_local_prev_ = iae_swarm_local_;
  }

  void find_global_best()
  {
    int i__ = 0;
    iae_swarm_global_.minCoeff(&i__);
    g_best_swarm_ = p_best_swarm_.col(i__);
  }
  void constrain_velocity_(int agent_)
  {
    // vel_max to be always less than 20 % of max search space
    double norm_vel_max_ = max_velocity_gain_ * sqrt(2 * pow(kp_max_, 2.000) + 2 * pow(kd_max_, 2.000));
    if (vel_swarm_.col(agent_).norm() > norm_vel_max_)
    {
      vel_swarm_.col(agent_) = vel_swarm_.col(agent_) / vel_swarm_.col(agent_).norm() * norm_vel_max_;
    }
  }


  void constrain_search_space_(int agent_)
  {
    for (auto row_ = 0; row_ < 4; row_++)
    {
      if (pos_swarm_.col(agent_)[0] < kp_min_)
      {
        pos_swarm_.col(agent_)[0] = kp_min_;
      }
      if (pos_swarm_.col(agent_)[1] < kd_min_)
      {
        pos_swarm_.col(agent_)[1] = kd_min_;
      }
      if (pos_swarm_.col(agent_)[2] < kp_min_)
      {
        pos_swarm_.col(agent_)[2] = kp_min_;
      }
      if (pos_swarm_.col(agent_)[3] < kd_min_)
      {
        pos_swarm_.col(agent_)[3] = kd_min_;
      }
      if (pos_swarm_.col(agent_)[0] > kp_max_)
      {
        pos_swarm_.col(agent_)[0] = kp_max_;
      }
      if (pos_swarm_.col(agent_)[1] > kd_max_)
      {
        pos_swarm_.col(agent_)[1] = kd_max_;
      }
      if (pos_swarm_.col(agent_)[2] > kp_max_)
      {
        pos_swarm_.col(agent_)[2] = kp_max_;
      }
      if (pos_swarm_.col(agent_)[3] > kd_max_)
      {
        pos_swarm_.col(agent_)[3] = kd_max_;
      }
    }
  }
  void update_omega_swarm_(int iter_)
  {
    omega_swarm_ = omega_swarm_max_ - (omega_swarm_max_ - omega_swarm_min_) / MAX_GENERATIONS * iter_;
  }

  void reset_oscillation_vars_() {
    oscillation_.error_rescaled_link_1_.Zero();
    oscillation_.error_rescaled_link_2_.Zero();
    oscillation_.index_ = 0;

  }
  void update_oscillation_vars() {
    oscillation_.error_rescaled_link_1_(oscillation_counter_) = error_joint_0;
    oscillation_.error_rescaled_link_2_(oscillation_counter_) = error_joint_1;
    oscillation_.error_rescaled_link_1_(oscillation_counter_) += error_joint_0;
  }

  int check_oscillation_(int counter_)
  {
    if (counter_ != 0)
    {
      update_oscillation_vars();
      if (oscillation_.index_ >= oscillation_.index_max_)
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
    reset_oscillation_vars_();
    return 0;
  }


  void fsm_state_0()
  {
    // Reset State FSM
    test_cycles_ = 2;  // seconds
    reset_ = 1;        // enable reset for hardware interface
    calc_error_ = 0;
    kp_1_ = 10;
    kp_2_ = 12;
    kd_1_ = 5;
    kd_2_ = 13;
    // default gains for reset
    state_ = 1;
    hardware_ = 1;  // interact with hardware
  }

  void fsm_state_1(int agent_, int iter__)
  {
    // pso -I step

    update_omega_swarm_(iter__);
    vel_swarm_.col(agent_) = vel_swarm_.col(agent_) + omega_swarm_ * vel_swarm_.col(agent_) +
                             c1_swarm_ * gamma_1_swarm_ * (p_best_swarm_.col(agent_) - pos_swarm_.col(agent_)) +
                             c2_swarm_ * gamma_2_swarm_ * (g_best_swarm_ - pos_swarm_.col(agent_));

    constrain_velocity_(agent_);

    pos_swarm_.col(agent_) = pos_swarm_.col(agent_) + vel_swarm_.col(agent_);
    constrain_search_space_(agent_);

    kp_1_ = pos_swarm_.col(agent_)(0);
    kd_1_ = pos_swarm_.col(agent_)(1);
    kp_2_ = pos_swarm_.col(agent_)(2);
    kd_2_ = pos_swarm_.col(agent_)(3);
    hardware_ = 0;  // dont interact with hardware
    state_ = 2;
  }

  void fsm_state_2()
  {
    // trajectory state fsm
    test_cycles_ = 6;  // seconds
    reset_ = 0;        // disable reset for hardware interface
    theta = 0;         // reset phase of trajectory
    calc_error_ = 1;   // calculate error IAE
    state_ = 3;
    hardware_ = 1;  // interact with hardware
  }
  void fsm_state_3(int agent_)
  {
    // pso -II step
    // update p_best and g_best
    if (iae_swarm_local_prev_ > iae_swarm_local_)
    {
      p_best_swarm_.col(agent_) = pos_swarm_.col(agent_);
    }
    iae_swarm_global_(agent_) = iae_swarm_local_;
    next_agent_ = 1;  // agent eval complete, move to next agent
    hardware_ = 0;    // dont interact with hardware
  }

  void elliptical_trajectory_generator()
  {
    theta = theta + omega * dt;
    //run oscillation_counter to soe cycles
    oscillation_counter_++;
    if (oscillation_counter_ == OSCILLATION_CYCLES_)
    {
      oscillation_counter_ = 0;
    }

    if (sin(theta) <= 0)
    {
      x_ = s_l_ * cos(theta);
      y_ = z_foot_;
      v_x_ = -s_l_ * omega * sin(theta);
      v_y_ = 0;
    }
    else
    {
      x_ = s_l_ * cos(theta);
      y_ = z_foot_ + w_h_ * sin(theta);
      v_x_ = -s_l_ * omega * sin(theta);
      v_y_ = w_h_ * omega * cos(theta);
    }
    if (reset_)
    {
      v_x_ = 0;
      v_y_ = 0;
      x_ = s_l_;
      y_ = z_foot_;
    }

    ee_vel_(0) = v_x_;
    ee_vel_(1) = v_y_;
    ee_pos_(0) = x_;
    ee_pos_(1) = y_;
    serial_2r_.inverseKinematics(ee_pos_, '<', joint_angles_);
    jac_inverse_ = serial_2r_.jacobianMatrix(joint_angles_).inverse();
    joint_velocities_ = jac_inverse_ * ee_vel_;
  }
  utils::Vector2d get_joints()
  {
    return joint_angles_;
  }
  utils::Vector2d get_velocities()
  {
    return joint_velocities_;
  }
};
}  // namespace autotuner_pd

#endif