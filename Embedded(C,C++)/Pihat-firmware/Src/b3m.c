

#include "main.h"
#include "b3m.h"



/*
 * @brief Verify the response agains the command that is sent
 *
 * @param motor_cmd: Motor command containing the command that was sent along with
 *                   the response received from the motor
 *
 * @retval -5 : Length error
 *         -6 : Command error
 *         -7 : Checksum error
 *         < -10: Error status received from motor
 */
int8_t verify_response(sMotorCmd* motor_cmd)
{
	uint8_t resp_len;
	uint8_t i;
	uint8_t sum = 0;

	resp_len = motor_cmd->resp[0];

	// Verify the length of the received data
	if(resp_len != motor_cmd->resp_size)
		return LEN_ERR;

	// Verify the response command
	if(motor_cmd->resp[1] != (motor_cmd->cmd[1] | 0x80))
		return RCMD_ERR;

	// Calculate the checksum
	for(i = 0; i < motor_cmd->resp_size-1; i++)
		sum += motor_cmd->resp[i];

	// Verify the checksum
	if(sum != motor_cmd->resp[motor_cmd->resp_size-1])
		return CHKSUM_ERR;


	// Verify the Error status received from the motor
	if((motor_cmd->cmd[2] == 0) && (motor_cmd->resp[2] != 0))
	    //Bit 0:SYSTEM STATUS, 1:MOTOR STATUS, 2:UART STATUS, 3:COMMAND STATUS
	    return -(motor_cmd->resp[2] + 10);

	// If all is well, return
	return 0;
}



/*
 * @brief Function to create a data packet for the motor READ operation
 *
 * @param id: Motor ID
 * @param addr: Address of the register
 * @param size: Number of bytes to be read from the motor
 * @param motor_cmd: Struct to store the data packet used to communicate with the motor
 *
 */
void b3mRead(uint8_t id, uint8_t addr, uint8_t size, sMotorCmd* motor_cmd)
{
	motor_cmd->cmd[0] = 0x07; 						// Size
	motor_cmd->cmd[1] = 0x03; 						//  Read command
	motor_cmd->cmd[2] = 0x3 + 0x80; 				// Option  ERROR STATUS +  CLEAR STATUS
	motor_cmd->cmd[3] = id;   						// ID of motor
	motor_cmd->cmd[4] = addr; 						// Address
	motor_cmd->cmd[5] = size;  						// Length
	motor_cmd->cmd[6] = motor_cmd->cmd[0] + motor_cmd->cmd[1] +
						motor_cmd->cmd[2] + motor_cmd->cmd[3] +
						motor_cmd->cmd[4] + motor_cmd->cmd[5]; // Checksum

	motor_cmd->cmd_size = 7;
	motor_cmd->resp_size = size + 5;

	return;
}

/*
 * @brief Function to create a data packet for the motor WRITE operation
 *
 * @param id: Motor ID
 * @param addr: Address of the register
 * @param len: Number of bytes to be read from the motor
 * @param buf: Buffer containing the data
 * @param motor_cmd: Struct to store the data packet used to communicate with the motor
 *
 */
void b3mWrite(uint8_t id, uint8_t addr, uint8_t len, uint8_t *buf, sMotorCmd* motor_cmd)
{
	uint8_t i;
	uint8_t sum = 0;

	motor_cmd->cmd[0] = 0x07 + len; 				// Size
	motor_cmd->cmd[1] = 0x04; 						// Command
	motor_cmd->cmd[2] = 0x0 + 0x80; 				// Option  ERROR STATUS +  CLEAR STATUS
	motor_cmd->cmd[3] = id;   						// ID of motor
	for(i = 0; i < len; i++)
	{
		motor_cmd->cmd[4+i] = buf[i]; 				// Contents to write
		sum += buf[i];
	}

	motor_cmd->cmd[4+len] = addr; 					// Address
	motor_cmd->cmd[5+len] = 1;  					// Count -- No. of devices
	motor_cmd->cmd[6+len] = motor_cmd->cmd[0] + motor_cmd->cmd[1] +
							motor_cmd->cmd[2] + motor_cmd->cmd[3] + sum +
							motor_cmd->cmd[4+len] + motor_cmd->cmd[5+len]; // Checksum

	motor_cmd->cmd_size = 7 + len;
	motor_cmd->resp_size = 5;

	return;
}

/*
 * @brief Function to create a data packet for the motor POSITION operation
 *
 * @param id  : List of Motor ID's
 * @param pos : List of motor positions
 * @param time: Time to move to the specified positino
 * @param len : Number of motors to command (this is same as the size of the id and pos list)
 * @param motor_cmd: Struct to store the data packet used to communicate with the motor
 *
 */
void b3mPosition(uint8_t* id, int16_t* pos, uint16_t time, uint8_t len, sMotorCmd* motor_cmd)
{
	uint8_t sum=0;

	motor_cmd->cmd[0] = 6 + 3*len; 					// Size
	motor_cmd->cmd[1] = 0x06; 					// Command
	motor_cmd->cmd[2] = 0x00; 					// Option
	for(int i=0; i<len; i++)
	{
		motor_cmd->cmd[3 + i*3] = id[i];   				// ID of motor
		motor_cmd->cmd[4 + i*3] = (0xFF & pos[i]); 		// POS_L
		motor_cmd->cmd[5 + i*3] = (0xFF & (pos[i] >> 8)); 	// POS_H
		sum += motor_cmd->cmd[3 + i*3] + motor_cmd->cmd[4 + i*3] + motor_cmd->cmd[5 + i*3];
	}
	motor_cmd->cmd[3+len*3] = (0xFF & time); 		// TIME_L
	motor_cmd->cmd[4+len*3] = (0xFF & (time >> 8)); // TIME_H
	motor_cmd->cmd[5+len*3] =   motor_cmd->cmd[0] + 		motor_cmd->cmd[1] +
								motor_cmd->cmd[2] + 		motor_cmd->cmd[3+len*3] +
								motor_cmd->cmd[4+len*3] + 	sum; // Checksum


	motor_cmd->cmd_size = 6 + 3*len;

	if(len == 1)
		motor_cmd->resp_size = 7;
	else
		motor_cmd->resp_size =0;

	return;
}

/*
 * @brief: To set the control mode of the motor
 *
 * @param id: Motor id
 * @param mode: Mode to set.
 *              0 : Position mode
 *              4 : Velocity mode
 *              8 : Current mode
 *              12: FF mode
 *
 */
void setMode(uint8_t id, uint8_t mode, sMotorCmd* motor_cmd)
{
	uint8_t addr 	= 0x28;
	uint8_t len 	= 1;
	uint8_t setmode = (0x0F & mode);

	b3mWrite(id, addr, len, &setmode, motor_cmd);
	return;
}


/*
 * @brief: To set the control mode of the motor
 *
 * @param id: Motor id
 * @param mode: Mode to set.
 *              0 : PRESET_POSITION
 *              1 : PRESET VELOCITY
 *              2 : PRESET TORQUE
 *
 */
void setGainPreset(uint8_t id, uint8_t gain_preset, sMotorCmd* motor_cmd)
{
	uint8_t addr   = 0x5C;
	uint8_t len	   = 1;
	uint8_t preset = (0x03 & gain_preset);

	b3mWrite(id, addr, len, &preset, motor_cmd);
	return;
}


