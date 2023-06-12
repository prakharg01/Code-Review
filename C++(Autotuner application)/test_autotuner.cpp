
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
 * 
 *  Author: Prakhar Goel
 */


#include "test_autotuner.h"

int main(int argc, char **argv)
{
  std::shared_ptr<EthComm> eth_comm = std::make_shared<EthComm>(PEER_IP, PEER_PORT, SELF_PORT);

  std::vector<MoteusCommand> cmds;
  std::vector<MoteusResponse> resp;

  std::vector<uint8_t> cmd_buffer;
  cmd_buffer.reserve(BUFFER_SIZE);

  const int num_joints = 2;
  std::map<std::string, int> name_motor_id_map;
  std::map<std::string, int> name_bus_id_map;
  std::vector<std::string> joint_names;

  joint_names.push_back("fl_abd_joint");
  joint_names.push_back("fl_hip_joint");
  joint_names.push_back("fl_knee_joint");

  name_motor_id_map = {
      {"fl_abd_joint", 1},
      {"fl_hip_joint", 2},
      {"fl_knee_joint", 3}
  };

  name_bus_id_map = {
      {"fl_abd_joint", 1},
      {"fl_hip_joint", 1},
      {"fl_knee_joint", 1}
  };

  cmds.resize(num_joints);
  resp.resize(num_joints);

  cmds[0] = {
      .id = 1,
      .position = 0.01,
      .velocity = 0.0,
      .feedforward_torque = 0,
      .kp_scale = 50,
      .kd_scale = 50,
      .watchdog_timeout = 0
  };

  cmds[1] = {
      .id = 2,
      .position = 0.01,
      .velocity = 0.0,
      .feedforward_torque = 0,
      .kp_scale = 50,
      .kd_scale = 50,
      .watchdog_timeout = 0
  };

  std::mutex mutex;

  // Initialize the Pi
  {

    std::lock_guard<std::mutex> lock(mutex);
    // Payload format: | NUM_JOINTS | JOINT1_M_ID | JOINT1_B_ID | JOINT2_M_ID | JOINT2_B_ID | ...
    serialize(cmd_buffer, reinterpret_cast<uint8_t *>(&num_joints), 1);
    for (auto i = 0; i < num_joints; i++)
    {
      uint8_t bus_id;
      uint8_t motor_id;

      motor_id = static_cast<uint8_t>(name_motor_id_map[joint_names[i]]);
      bus_id = static_cast<uint8_t>(name_bus_id_map[joint_names[i]]);
      serialize(cmd_buffer, reinterpret_cast<uint8_t *>(&motor_id), 1);
      serialize(cmd_buffer, reinterpret_cast<uint8_t *>(&bus_id), 1);
    }

    int ret;
    ret = eth_comm->transfer(static_cast<uint8_t>(eCmdMotorInit), cmd_buffer, 2500);
    if (ret < 0)
    {
      printf("Error: Unable to initialize the Pi. Code: %d \n", ret);
      exit(-1);
    }
  }

  // Send a stop command to ensure the motors
  // recover from any fault condition
  {
    int ret;
    cmd_buffer.clear();
    ret = eth_comm->transfer(static_cast<uint8_t>(eCmdMotorStop), cmd_buffer, 2500);
    if (ret < 0)
    {
      printf("Error: Unable to send the stop command to the Pi. Code: %d \n", ret);
      exit(-1);
    }
  }

  // Wait for the stop command to be executed
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  {
    std::lock_guard<std::mutex> lock(mutex);
    std::shared_ptr<autotuner_pd::autotuner> autotuner_ = std::make_shared<autotuner_pd::autotuner>();
  }
  autotuner_->init_fsm_();

  // Start autotuner application
  for (auto k = 0; k < MAX_GENERATIONS; k++)
  {
    std::cout << std::endl << "Generation #" << k;

    autotuner_->find_global_best();

    for (auto agent_ = 0; agent_ < MAX_AGENTS; agent_++)
    {
      autotuner_->reset_fsm_();  // reset fsm for next agent

      // fsm:- {0->1->2->3}->...next agent
      while (!autotuner_->next_agent_)
      {
        std::lock_guard<std::mutex> lock(mutex);
        switch (autotuner_->state_)
        {
          case 0:
          {
            autotuner_->fsm_state_0();
            break;
          }
          case 1:
          {
            autotuner_->fsm_state_1(static_cast<int>(agent_), static_cast<int>(k));
            break;
          }
          case 2:
          {
            autotuner_->fsm_state_2();
            break;
          }
          case 3:
          {
            autotuner_->fsm_state_3(static_cast<int>(agent_));
            break;
          }
          default:
          {
            std::cout << std::endl << "Terminating process :: error state" << std::endl;
            exit(-1);
          }
        }

        if (autotuner_->hardware_) // dont interact for state 1 & 3 of agent i in FSM
        {
          // hardware interaction (interact for test_cycles seconds)
          const int TEST_CYCLES = static_cast<int>(autotuner_->test_cycles_ / autotuner_->dt);
          for (auto i = 0; i < TEST_CYCLES; i++)
          {
            autotuner_->elliptical_trajectory_generator();
            cmds[0].position = autotuner_->get_joints()[0] * 3 / M_PI;
            cmds[0].velocity = autotuner_->get_velocities()[0] * 3 / M_PI;
            cmds[1].position = autotuner_->get_joints()[1] * 4.09 / M_PI;
            cmds[1].velocity = autotuner_->get_velocities()[1] * 4.09 / M_PI;

            cmds[0].kp_scale = autotuner_->kp_1_;
            cmds[0].kd_scale = autotuner_->kd_1_;
            cmds[1].kp_scale = autotuner_->kp_2_;
            cmds[1].kd_scale = autotuner_->kd_2_;

            // WRITE
            {
              cmd_buffer.clear();
              for (const auto& cmd : cmds)
                serialize(cmd_buffer, reinterpret_cast<const uint8_t *>(&cmd), sizeof(MoteusCommand));

              int ret;
              ret = eth_comm->transfer(static_cast<uint8_t>(eCmdMotorCmd), cmd_buffer, 1500);
              if (ret <= 0)
                std::cout << "Error: failed to get a response for motor command." << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD_MS));

            // READ
            {
              auto start = std::chrono::high_resolution_clock::now();
              int ret;

              cmd_buffer.clear();
              ret = eth_comm->transfer(static_cast<uint8_t>(eCmdMotorState), cmd_buffer, 1500);
              if (ret < 0)
              {
                std::cout << "Error: Did not receive a response containing the motor state." << std::endl;
                continue;
              }

              auto stop = std::chrono::high_resolution_clock::now();
              auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

              std::cout << "Execution duration is: " << duration.count() << std::endl;

              for (const auto& cmd : cmds)
              {
                if (cmd_buffer.size() < sizeof(MoteusResponse))
                {
                  std::cout << "Error: Insufficient data in motor state." << std::endl;
                  break;
                }
                deserialize(cmd_buffer, reinterpret_cast<uint8_t *>(&resp[j]), sizeof(MoteusResponse));
                if (autotuner_->calc_error_ == 1)
                {
                  // calculate IAE
                  autotuner_->iae_swarm_local_ += std::abs(autotuner_->get_joints()[0] - (resp[j].position)[0]) +
                                                 std::abs(autotuner_->get_joints()[1] - (resp[j].position)[1]);
                  autotuner_->error_joint_0 =
                      autotuner_->get_joints()[0] - (resp[j].position)[0];
                  autotuner_->error_joint_1 =
                      autotuner_->get_joints()[1] - (resp[j].position)[1];

                  // calculate constraints
                  // int ret_ = autotuner_->check_oscillation(autotuner_->oscillation_counter_);
                  // if (ret_)
                  // {
                  //   autotuner_->iae_swarm_local_ = e_max_;
                  // }
                  int ret_=0;
                  if ((std::abs(autotuner_->error_joint_0) >= autotuner_->e_max_ && static_cast<int>(i) > TEST_CYCLES / 6) ||
                      (std::abs(autotuner_->error_joint_1) >= autotuner_->e_max_ && static_cast<int>(i) > TEST_CYCLES / 6) ||
                      ret_)
                  {
                    autotuner_->vel_swarm_.col(agent_) = autotuner_->vel_swarm_.col(agent_).Zero();  // set velocity to zero

                    // Send a stop command to ensure the motors recover from any fault condition
                    {
                      int ret;
                      cmd_buffer.clear();
                      ret = eth_comm->transfer(static_cast<uint8_t>(eCmdMotorStop), cmd_buffer, 2500);
                      if (ret < 0)
                      {
                        printf("Error: Unable to send the stop command to the Pi. Code: %d \n", ret);
                        exit(-1);
                      }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // wait for 100 ms for stop to execute
                  }
                }
                  std::cout << "Response:: ID:" << (int)resp[j].id << " Position:" << resp[j].position
                            << " Torque:" << resp[j].torque << std::endl;
                }
              }
            }
          }
        }
        std::cout << std::endl << "agent # " << agent_ << "complete";
        // log the values
        autotuner_->fout << k << ", " << agent_ << ", " << autotuner_->iae_swarm_local_ << ", " < < < <
            ", " << kp_1_ << ", " << kd_1_ << ", " << kp_2_ << ", " << kd_2_ << ", " << 0 << ", " << 0 << ", " << 0
                 << ", " << 0 << ", " << 0 << ", " << 0 << "\n";
      }
    std::cout << std::endl << "generation #" << k << "complete";
  }

  return 0;
}
