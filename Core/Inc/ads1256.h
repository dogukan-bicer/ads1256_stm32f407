
#ifndef INC_ADS1256_H_
#define INC_ADS1256_H_

#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "math.h"

/*____ USER DEFINE BEGIN _____________________*/

/*CHIP_SELECT define group*/
#define CS_ACTIVE		 GPIO_PIN_RESET
#define CS_DEACTIVE 	 GPIO_PIN_SET

// multiplexer codes
#define ADS125X_MUXP_AIN0 0x00
#define ADS125X_MUXP_AIN1 0x10
#define ADS125X_MUXP_AIN2 0x20
#define ADS125X_MUXP_AIN3 0x30
#define ADS125X_MUXP_AIN4 0x40
#define ADS125X_MUXP_AIN5 0x50
#define ADS125X_MUXP_AIN6 0x60
#define ADS125X_MUXP_AIN7 0x70
#define ADS125X_MUXP_AINCOM 0x80

#define ADS125X_MUXN_AIN0 0x00
#define ADS125X_MUXN_AIN1 0x01
#define ADS125X_MUXN_AIN2 0x02
#define ADS125X_MUXN_AIN3 0x03
#define ADS125X_MUXN_AIN4 0x04
#define ADS125X_MUXN_AIN5 0x05
#define ADS125X_MUXN_AIN6 0x06
#define ADS125X_MUXN_AIN7 0x07
#define ADS125X_MUXN_AINCOM 0x08

/*PGA_SET_VAL define group*/
#define PGA_1		0x0
#define PGA_2		0x1
#define PGA_4		0x2
#define PGA_8		0x3
#define PGA_16		0x4
#define PGA_32		0x5
#define PGA_64		0x6

/*DRATE_SET_VAL define group*/

#define DRATE_2_5 			0b00000011
#define DRATE_5				0b00010011
#define DRATE_10			0b00100011
#define DRATE_15			0b00110011
#define DRATE_25			0b01000011
#define DRATE_30			0b01010011
#define DRATE_50			0b01100011
#define DRATE_60			0b01110010
#define DRATE_100			0b10000010
#define DRATE_500			0b10010010
#define DRATE_1000			0b10100001
#define DRATE_2000 			0b10110000
#define DRATE_3750			0b11000000
#define DRATE_7500			0b11010000
#define DRATE_15000			0b11100000
#define DRATE_30000			0b11110000

/*______USER DEFINE END__________________________________________________________*/


/* ADS1256 commands */
#define WAKEUP_CMD		0x0
#define READ_DATA_CMD 	0x1
/*start/stop command of read data continuously */
#define RDATAC_CMD		0x3
#define SDATAC_CMD		0xf
/* R/W reagisters cmd */
#define READ_REG_CMD	0x10
#define WRITE_REG_CMD	0x50

#define SELF_CAL_CMD	0xf0
#define SYNC_CMD		0xfc
#define RESET_CMD		0xfe

#define NULL_CMD2		0x0

/* ADS1256 registers map */
#define STATUS_REG		0x0
#define MUX_REG			0x1
#define ADCON_REG		0x2
#define DRATE_REG		0x3
#define IO_REG			0x4
/* offset comapensation registers */
#define OFC0_REG		0x5
#define OFC1_REG		0x6
#define OFC2_REG		0x7
/* offset comapensation registers */
#define FSC0_REG		0x8
#define FSC1_REG		0x9
#define FSC2_REG		0xA

typedef enum {
	ANALOG=0,
	ADC_REF,
	EXT_REF,
	LAST_CHANNEL
}channel_nr_t;



/*
 * Description	Resource	Path	Location	Type
./Core/Src/main.o:D:/ADS1256/stm32_proje/ads1256_template/Debug/../Core/Inc/ads1256.h:46: multiple definition of `drv_ready';
./Core/Src/ads1256.o:D:/ADS1256/stm32_proje/ads1256_template/Debug/../Core/Inc/ads1256.h:46: first defined here	ads1256_template
 	C/C++ Problem
 * */

void ads_wakeup();
void ads_sync();
void ads_reset();

void ads_stop_contunue_data ();
void ads_start_contunue_data ();

uint8_t ads_read_register(uint8_t reg);
void ads_write_register(uint8_t reg,uint8_t value);
void ads_self_cal();
uint32_t ads_read_data();

bool spi_free();



/* high level functions - public members */
void ads_init();
bool ads_change_channel(int8_t p_chan, int8_t n_chan);
void ads_perform_self_calib();
channel_nr_t ads_get_selected_adc_channel();
float get_ref_temp( channel_nr_t channel, float voltage);


/*_____USER CODE FUNCTION PROTOTYPES*/

/*SubPrototypes*/
void set_PGA(uint8_t PGA);
void set_DATARATE(uint32_t datarate);
void printMessage(char *format, ...);



#endif /* INC_ADS1256_H_ */

