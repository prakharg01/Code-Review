

#include "main.h"
#include "spi_comm.h"
#include "register_bank.h"


SPI_HandleTypeDef* hspi;

typedef enum {
	eCmdNone,
	eCmdRead,
	eCmdWrite
} eSPICmds;

typedef enum {
	eStateNone,
	eStateInit,
	eStateRun,
}eFSMStates;


typedef struct {
	uint16_t 	num_read; 	// Number of bytes read
	uint16_t 	num_write; 	// Number of bytes written
	eSPICmds	cmd;		// Command received from master
	uint8_t		address;	// Address of the registers to read/write
}sSPIComm;



// Global variables to store the state.
// These must be accessed only from this file
static sSPIComm spi_comm;
static eFSMStates fsm_state = eStateNone;


void setStateRun() {
	fsm_state = eStateRun;
	__HAL_SPI_ENABLE(hspi);
	__HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
}


void setStateInit(){ fsm_state = eStateInit; }


void initCommFSM(SPI_HandleTypeDef* hspi_in) {
	hspi = hspi_in;
	hspi->Instance->DR = (uint16_t) 0x0;
	//__HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
	return;
}

void resetCommFSM(void){
	spi_comm.num_read   = 0;
	spi_comm.num_write 	= 0;
	spi_comm.cmd		= eCmdNone;
	spi_comm.address	= 0;

	release_sensor_register_lock();
	release_actuator_register_lock();
}


void runCommFSM (void) {

	uint16_t clear_data=0;
	uint8_t data = 0;
	uint8_t *addr;
	uint8_t	read_addr, write_addr;


	switch(fsm_state){

	case eStateInit:
		// Clear all Rx and Tx FIFOs by reading all 4 bytes and writing to all 4 bytes
		clear_data = (uint16_t) hspi->Instance->DR;
		clear_data = (uint16_t) hspi->Instance->DR;
		UNUSED(clear_data); // To avoid GCC warning
		hspi->Instance->DR = (uint16_t) 0x0000;
		hspi->Instance->DR = (uint16_t) 0x0000;

		__HAL_SPI_DISABLE(hspi);

		resetCommFSM();
		break;

	case eStateRun:
		// If received interrupt from SPI, then process it
		addr = (uint8_t *)&hspi->Instance->DR;
		data = *addr;
		if(spi_comm.num_read == 0) {
			spi_comm.cmd = data;
			*addr = data;
			spi_comm.num_read++;
		} else if(spi_comm.num_read == 1) {
			spi_comm.address = data;
			*addr = data;
			spi_comm.num_read++;
		} else {
			switch(spi_comm.cmd) {
			case eCmdRead:
				take_sensor_register_lock();
				read_addr = spi_comm.address + spi_comm.num_read -2;
				if(read_addr < SENSOR_REGISTER_SIZE)
					*addr = read_sensor_data(read_addr);
				else
					*addr = 0;
				spi_comm.num_read++;
				break;
			case eCmdWrite:
				take_actuator_register_lock();
				write_addr = spi_comm.address + spi_comm.num_write;
				if(write_addr < ACTUATOR_REGISTER_SIZE) {
					write_actuator_data(spi_comm.address + spi_comm.num_write,  data);
					spi_comm.num_write++;
				}
				break;
			default:
				break;
			}
		}
		break;

	default:
		break;
	};
}



