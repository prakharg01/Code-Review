#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/spi/spidev.h>
#include <linux/types.h>

#define FILE_ERROR -1
#define MODE_ERROR -2
#define BITS_ERROR -3
#define SPEED_ERROR -4

class SPIInterface 
{

  const uint8_t READ_CMD = 1;
  const uint8_t WRITE_CMD = 2;

  int     fd_;
  uint8_t mode_;
  uint8_t bits_;
  uint32_t speed_;

  int transfer_(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t size)
  {
    int ret;
    uint16_t delay = 0;

    struct spi_ioc_transfer tr={0};
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_buf;
    tr.len = size;
    tr.delay_usecs = delay;
    tr.speed_hz = speed_;
    tr.bits_per_word = bits_;

    ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
      return -1;

    return 0;
  }

  public:

  SPIInterface(){}
  ~SPIInterface()
  {
    if(fd_)
      close(fd_);
  }

  int init(char* device, uint8_t mode, uint8_t bits, uint32_t speed)
  {
    int ret = 0;

    mode_ = mode;
    bits_ = bits;
    speed_ = speed;

    fd_ = open(device, O_RDWR);
    if (fd_ < 0){
      fd_ = 0;
      return FILE_ERROR;
    }

    /*
     * spi mode
     */
    ret = ioctl(fd_, SPI_IOC_WR_MODE, &mode_);
    if (ret == -1)
      return MODE_ERROR;

    ret = ioctl(fd_, SPI_IOC_RD_MODE, &mode_);
    if (ret == -1)
      return MODE_ERROR;

    /*
     * bits per word
     */
    ret = ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_);
    if (ret == -1)
      return BITS_ERROR;

    ret = ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_);
    if (ret == -1)
      return BITS_ERROR;

    /*
     * max speed hz
     */
    ret = ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_);
    if (ret == -1)
      return SPEED_ERROR;

    ret = ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_);
    if (ret == -1)
      return SPEED_ERROR;

    return ret;
  }

  int read(uint8_t addr, uint8_t *rx_buf, uint8_t size)
  {
    uint8_t rx[256]={0};
    uint8_t tx[256]={0};
    int ret = 0;

    tx[0] = READ_CMD;
    tx[1] = addr;

    ret = transfer_(tx, rx, size+4);
    if(ret < 0)
    {
      printf("read: failed transaction.\n");
      return -1;
    }
    
    if(rx[0] != 0 || rx[1] != 0 || rx[2] != READ_CMD || rx[3] != addr)
      return -1;

    for(int i=0; i<size; i++)
      rx_buf[i] = rx[i+4];

    return size;
  }

  int write(uint8_t addr, uint8_t *tx_buf, uint8_t size)
  {
    uint8_t rx[256]={0};
    uint8_t tx[256]={0};
    int ret=0;

    tx[0] = WRITE_CMD;
    tx[1] = addr;

    for(int i=0; i<size; i++)
      tx[i+2] = tx_buf[i];

    ret = transfer_(tx, rx, size+2);
    if(ret < 0)
    {
      printf("write: failed transaction.\n");
      return -1;
    }
 
    if(rx[0] != 0 || rx[1] != 0 || rx[2] != WRITE_CMD || rx[3] != addr)
      return -1;

    return size;
  }


};

int main(int argc, char** argv)
{
  char device[] = "/dev/spidev0.0";

  const uint8_t  bits = 8;
  const uint32_t speed = 5000000;
  const uint8_t  mode = 0 ;

  int ret =0;

  uint8_t rx[256]={0};
  uint8_t tx[256] = {0};

  SPIInterface spi_interface;
  ret = spi_interface.init(device, mode, bits, speed);
  if(ret < 0)
  {
    printf("Unable to open device: %s, err: %d\n", device, ret);
    return -1;
  }

  ret = spi_interface.read(140, rx, 12);
  if(ret < 0)
    printf("SPI read error.\n");

  for(int i=0; i<12; i++)
  {
    if(i%8 == 0)
      printf("\n");
    printf("%02X  ", rx[i]);
  }
  printf("\n");


  for(int i=0;i<10;i++)
    tx[i] = 5*i;

  ret = spi_interface.write(0x05, tx, 10);
  if(ret < 0)
    printf("SPI write error.\n");



  return 0;
}
