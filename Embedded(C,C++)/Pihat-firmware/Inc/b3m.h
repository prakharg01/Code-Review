

#ifndef INC_B3M_H_
#define INC_B3M_H_


#define RECV_TIMEOUT_ERR    -1
#define LEN_ERR 	-5
#define RCMD_ERR 	-6
#define CHKSUM_ERR 	-7

// OPERATION MODES
#define NORMAL_MODE 	0
#define FREE_MODE 	2
#define HOLD_MODE 	3

// CONTROL MODES
#define POSITION_MODE 	0
#define VELOCITY_MODE 	4
#define CURRENT_MODE 	8
#define FF_MODE 	12

// PRESET
#define PRESET_POSITION 0
#define PRESET_VELOCITY 1
#define PRESET_TORQUE   2


/*
 * @brief Structure to hold the data related to a motor command
 */

typedef struct{
	uint8_t cmd[256];
	uint8_t cmd_size;
	uint8_t resp[256];
	uint8_t resp_size;
}sMotorCmd;


int8_t verify_response(sMotorCmd* motor_cmd);

void b3mRead(uint8_t id, uint8_t addr, uint8_t size, sMotorCmd* motor_cmd);
void b3mWrite(uint8_t id, uint8_t addr, uint8_t len, uint8_t *buf, sMotorCmd* motor_cmd);
void b3mPosition(uint8_t* id, int16_t* pos, uint16_t time, uint8_t len, sMotorCmd* motor_cmd);

void setMode(uint8_t id, uint8_t mode, sMotorCmd* motor_cmd);
void setGainPreset(uint8_t id, uint8_t gain_preset, sMotorCmd* motor_cmd);

#endif /* INC_B3M_H_ */
