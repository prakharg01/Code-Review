

#ifndef SRC_SPI_COMM_H_
#define SRC_SPI_COMM_H_

void initCommFSM(SPI_HandleTypeDef*);
void runCommFSM (void);

void setStateRun();
void setStateInit();

#endif /* SRC_SPI_COMM_H_ */
