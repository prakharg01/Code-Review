
#include "stdbool.h"

#include "main.h"
#include "register_bank.h"
#include "b3m_task.h"
#include "b3m.h"
#include "timer.h"

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;

#define NUM_MOTORS 12
#define NUM_TASKS 2

#define MOTOR_READ_ADDRESS	0x2A
#define MOTOR_READ_SIZE		34

#define STATUS_OK		(0     )
#define STATUS_TIMEOUT 	(1 << 0)
#define STATUS_RESP_ERR (1 << 1)

uint32_t last_read_tick_motor[NUM_TASKS]={0};

/*
 * @brief Indicates the states of the State-machine
 * used in the Read task.
 */
typedef enum{
	eSEND_CMD,
	eRECV_RESP,
}eState;


/*
 * @brief Structure to hold the motor state data
 *
 * @param status: 1 -> Timeout;
 */
typedef struct{
	uint8_t  id;
	uint8_t  status;
	int16_t position;
	int16_t velocity;
	int16_t current;
	uint16_t temperature;
	uint16_t voltage;
}sMotorData;

/*
 * @brief Structure to hold the data related to the task
 */
typedef struct {
	UART_HandleTypeDef* huart;
	uint8_t num_motors;
	uint8_t current_motor_num;
	eState current_state;
	bool response_received;
	sMotorData motor_data[12];
	sMotorCmd motor_cmd;
}sMotorTask;


typedef enum{
	ePOSITION,
	eVELOCITY,
	eCURRENT,
	eVOLTAGE,
	eTEMPERATURE
}eMotorDataType;

const uint8_t motor_id_list[NUM_MOTORS] = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43};
sMotorTask motor_tasks[NUM_TASKS];
bool task_complete[NUM_TASKS]={false};

sMotorCmd motor_write_cmd={0};

// Function prototypes
void motors_read_task(uint8_t task_id);
void motor_update_registers();
sMotorData* find_motor_data(uint8_t id);
uint16_t concatenateData(uint8_t lo_byte, uint8_t hi_byte);

/*
 * @brief Initialize the motor tasks
 */
void motors_init()
{
	// Update the motor_task struct with the details
	motor_tasks[0].huart = &huart3;
	motor_tasks[0].num_motors = NUM_MOTORS/2;
	motor_tasks[0].current_motor_num = 0;
	motor_tasks[0].current_state = eSEND_CMD;
	motor_tasks[0].motor_data[0].id  = motor_id_list[3];
	motor_tasks[0].motor_data[1].id  = motor_id_list[4];
	motor_tasks[0].motor_data[2].id  = motor_id_list[5];
	motor_tasks[0].motor_data[3].id  = motor_id_list[6];
	motor_tasks[0].motor_data[4].id  = motor_id_list[7];
	motor_tasks[0].motor_data[5].id  = motor_id_list[8];

	motor_tasks[1].huart = &huart1;
	motor_tasks[1].num_motors = NUM_MOTORS/2;
	motor_tasks[1].current_motor_num = 0;
	motor_tasks[1].current_state = eSEND_CMD;
	motor_tasks[1].motor_data[0].id  = motor_id_list[0];
	motor_tasks[1].motor_data[1].id  = motor_id_list[1];
	motor_tasks[1].motor_data[2].id  = motor_id_list[2];
	motor_tasks[1].motor_data[3].id  = motor_id_list[9];
	motor_tasks[1].motor_data[4].id  = motor_id_list[10];
	motor_tasks[1].motor_data[5].id  = motor_id_list[11];
	// Initialize task_complete flags
	for(int i=0; i<NUM_TASKS; i++)
		task_complete[i] = false;

	// Set motors in position control mode
	for(int i = 0; i<NUM_TASKS; i++)
	{
		sMotorTask* task = &motor_tasks[i];
		sMotorCmd* motor_cmd = &task->motor_cmd;
		for(int j=0; j<task->num_motors; j++)
		{
			uint8_t id = task->motor_data[j].id;
			setMode(id, FREE_MODE, motor_cmd);
			HAL_UART_Transmit(task->huart, motor_cmd->cmd, motor_cmd->cmd_size, 5);

			HAL_Delay(10);

			setMode(id, FREE_MODE | POSITION_MODE, motor_cmd);
			HAL_UART_Transmit(task->huart, motor_cmd->cmd, motor_cmd->cmd_size, 5);

			HAL_Delay(10);

			setGainPreset(id, PRESET_POSITION, motor_cmd);
			HAL_UART_Transmit(task->huart, motor_cmd->cmd, motor_cmd->cmd_size, 5);

			HAL_Delay(10);

			setMode(id, NORMAL_MODE | POSITION_MODE, motor_cmd);
			HAL_UART_Transmit(task->huart, motor_cmd->cmd, motor_cmd->cmd_size, 5);

			HAL_Delay(10);
			HAL_UART_AbortReceive(task->huart);
			HAL_Delay(10);
		}
	}
	//HAL_TIM_Base_Start(&htim1);

}

/*
 * @brief Run all the tasks to read the motors
 */
void motors_read()
{
	for(int i=0; i<NUM_TASKS; i++)
		motors_read_task(i);

	motor_update_registers();
	return;
}

/*
 * @brief Task to read the state of the motor
 *
 * @note Multiple tasks can exist to allow for parallel communication with
 * motors on different UART buses
 *
 * @param task_id: ID of the task that needs to be executed.
 */
void motors_read_task(uint8_t task_id)
{
	uint32_t current_tick_motor;
	sMotorTask* task;
	sMotorCmd* motor_cmd;
	uint8_t id;

	// Invalid input id
	if(task_id >= NUM_TASKS)
		return;

	if(task_complete[task_id])
		return;

	task = &motor_tasks[task_id];
	motor_cmd = &task->motor_cmd;

	switch(task->current_state)
	{
	case eSEND_CMD:
		// Send a motor command to read the state
		id = task->motor_data[motor_tasks->current_motor_num].id;
		b3mRead(id, MOTOR_READ_ADDRESS, MOTOR_READ_SIZE, motor_cmd);

		task->response_received = false;

		HAL_UART_Receive_DMA(task->huart, motor_cmd->resp, motor_cmd->resp_size);
		HAL_UART_Transmit_DMA(task->huart, motor_cmd->cmd, motor_cmd->cmd_size);

		// Start the timer/counter
		//__HAL_TIM_SET_COUNTER(&htim1,0);

		last_read_tick_motor[task_id]=get_tick_micro();

		// Task has started and it is not yet complete
//		task_complete[task_id] = false;

		// Update state for subsequent call to function
		task->current_state = eRECV_RESP;
		break;

	case eRECV_RESP:

		// If response has not arrived, then TIMEOUT
		if (task->response_received)
		{
			// Check the validity of the response
			if(verify_response(motor_cmd) < 0)
			{
				task->motor_data[task->current_motor_num].status		= STATUS_RESP_ERR;
			}
			else
			{
				task->motor_data[task->current_motor_num].position 		= concatenateData(motor_cmd->resp[2  + 4], motor_cmd->resp[3  + 4]);
				task->motor_data[task->current_motor_num].velocity 		= concatenateData(motor_cmd->resp[8  + 4], motor_cmd->resp[9  + 4]);
				task->motor_data[task->current_motor_num].current 		= concatenateData(motor_cmd->resp[30 + 4], motor_cmd->resp[31 + 4]);
				task->motor_data[task->current_motor_num].temperature 	= concatenateData(motor_cmd->resp[28 + 4], motor_cmd->resp[29 + 4]);
				task->motor_data[task->current_motor_num].voltage 		= concatenateData(motor_cmd->resp[32 + 4], motor_cmd->resp[33 + 4]);
				task->motor_data[task->current_motor_num].status		= STATUS_OK;
			}
		}
		else
		{
			// Check for timeout
			current_tick_motor=get_tick_micro();
			if(current_tick_motor-last_read_tick_motor[task_id] > 50)
			{
				task->motor_data[task->current_motor_num].position 		= 0;
				task->motor_data[task->current_motor_num].velocity 		= 0;
				task->motor_data[task->current_motor_num].current 		= 0;
				task->motor_data[task->current_motor_num].temperature 	= 0;
				task->motor_data[task->current_motor_num].voltage 		= 0;
				task->motor_data[task->current_motor_num].status		= STATUS_TIMEOUT;

				HAL_UART_AbortReceive(task->huart);
			}
			else
			{
				// If not timed-out then return and wait for the next pass to process
				// the data.
				return;
			}
		}

		// All motors covered; mark the task as complete
		if(task->current_motor_num == (task->num_motors - 1))
			task_complete[task_id] = true;

		// Move to the next motor
		task->current_motor_num = (task->current_motor_num + 1) % task->num_motors;

		// Update state for next subsequent call to function
		task->current_state = eSEND_CMD;
		break;

	default:
		break;
	}
}

/*
 * @brief Update the register bank with the motor state
 *
 * The motor state will be updated once all the tasks are complete.
 */
void motor_update_registers()
{
	sMotorData* motor_data;
	bool tasks_complete = true;

	for(int i=0; i<NUM_TASKS; i++)
		tasks_complete = tasks_complete && task_complete[i];

	if(!tasks_complete)
		return;

	for (int i=0; i<NUM_MOTORS; i++)
	{
		motor_data = find_motor_data(motor_id_list[i]);
		if(motor_data)
			write_sensor_data(MOTOR_ADDR + i*MOTOR_DATA_NUM_BYTES,(uint8_t *) motor_data, MOTOR_DATA_NUM_BYTES);
	}
	for(int i = 0; i<NUM_TASKS; i++){
		task_complete[i] = false;
	}
}

/*
 * @brief Find the data corresponding to a specific motor whose ID is provided
 *
 * @param id: ID of the motor to search
 *
 * @retval NULL if data is not found, pointer to the data if it is found
 */
sMotorData* find_motor_data(uint8_t id)
{
	sMotorData* motor_data = 0;

	for(int t=0; t<NUM_TASKS; t++)
	{
		for(int i=0; i < motor_tasks[t].num_motors; i++)
		{
			if(motor_tasks[t].motor_data[i].id == id)
			{
				motor_data = &motor_tasks[t].motor_data[i];
				return motor_data;
			}
		}
	}
	return 0;
}

/*
 * @brief Task to read the register bank and send the commands to the motors
 */
void motors_write()
{
	uint8_t buf[38]={0};
	uint8_t id[NUM_MOTORS];
	uint16_t pos[NUM_MOTORS];
	uint16_t time;

	sMotorTask* task;
	sMotorCmd* motor_cmd;

	// Check for new data in the registers
	if(!is_new_data())
		return;

	// Read the buffer
	if(!read_actuator_data(0, 38, buf))
		return;

	time = ((uint16_t) buf[36] << 8) + buf[37];

	uint32_t ticks_1 = get_tick_micro();

	for(int i =0; i<NUM_TASKS; i++)
	{
		task = &motor_tasks[i];
		motor_cmd = &task->motor_cmd;

		// Create MultiPos command
		for(int j=0; j < task->num_motors; j++)
		{
			for(int k=0; k<NUM_MOTORS; k++)
			{
				id[j] = 0;
				pos[j]= 0;
				if(buf[3*k] == task->motor_data[j].id)
				{
					id[j] = buf[3*k];
					pos[j] = ((uint16_t) buf[3*k + 1] << 8) + buf[3*k+2];
					break;
				}
			}
		}
		b3mPosition(id, (int16_t *) pos, time , task->num_motors, motor_cmd);
		HAL_UART_Transmit_DMA(task->huart, motor_cmd->cmd, motor_cmd->cmd_size);
	}
	uint32_t ticks = get_tick_micro();
	return;
}

/*
 * @brief Callback function for the Rx complete event
 *
 * @note This function overrides the function of the same
 * name which is defined in the HAL UART file but defined as weak.
 *
 * @param huart: Handle for the UART from which the event is raised.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	for(int i=0; i< NUM_TASKS; i++)
	{
		if(huart == motor_tasks[i].huart)
		{
			motor_tasks[i].response_received = true;
			return;
		}
	}
	return;
}




/*
 * @brief Combine two bytes received from the B3M motor into a 16-bit word
 *
 * @param lo_byte: Lower 8-bits of the data
 * @param hi_byte: Higer 8-bits of the data
 *
 * @retval Returns the concatenated data
 */
inline uint16_t concatenateData(uint8_t lo_byte, uint8_t hi_byte)
{
	return  ((uint16_t) (hi_byte << 8)) + lo_byte;
}


