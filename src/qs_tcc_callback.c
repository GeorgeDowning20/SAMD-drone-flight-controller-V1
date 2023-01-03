#include <asf.h>
#include "string.h"
#include "stdio.h"
#include "arm_math.h"
#include "math.h"
#include <string.h>

//#include "conf_example.h"

//struct PID_Struct{
	//float KP;
	//float KI;
	//float KD;
	//float Prev_Error;
	//float Integral_Error;
	//
//} ;

//test test this is rhw the motor control one




struct tcc_module tcc_instance;
enum status_code stat = STATUS_OK;

struct tcc_module tcc_one_instance, tcc_zero_instance;

void configure_usart(void);
void configu_eic(void);
void configu_tcc(void);
void configure_evsys (void);

void printdata(char * header, uint16_t printint);

//float PID_Output(float Input, float Target, struct PID_Struct * PID);

/* 
 * Function to configure usart, eic,tcc and event system. 
 * configures the usart with CONF_BAUD_RATE baud rate.
 */
//struct usart_module usart_instances;
struct tcc_module tcc_instances;
struct events_resource event_resources;

volatile uint8_t newkeydata=0;

uint32_t pulse_width;
uint8_t newsensordata=0;

float64_t motor1_pwm, motor2_pwm, motor3_pwm;
uint16_t demanded_thrust=0;

uint8_t newbno005data=0;

uint8_t messagestring[] = "Q 000000\r\n";
uint8_t binarystring[]="B 00000000 \r\n";
uint8_t digit;

static struct usart_module cdc_instance;

static struct usart_module cdc_instance_BNO005;

static struct usart_module cdc_instance_MATEK;

uint8_t writetoBNObuffer[32];

uint8_t rx_buffer[1];
uint8_t tx_buffer[3]={0, 0x0d, 0x0a};
#define MAX_RX_BUFFER_LENGTH 1


uint8_t BNO005_tx_buffer[3];
uint8_t BNO005_rx_buffer[1];
#define MAX_RX_BNO005_LENGTH 1

uint8_t MATEK_tx_buffer[3];
uint8_t MATEK_rx_buffer[1];
#define MAX_RX_MATEK_LENGTH 1

uint8_t read_flysky_state;
uint8_t read_flysky_state2;

uint8_t Channel1_lsb,Channel1_msb, Channel2_lsb, Channel2_msb, Channel3_lsb,Channel3_msb,Channel4_lsb,Channel4_msb;
uint16_t Channel1, Channel2, Channel3, Channel4;


#define CHIP_ID 0x00				//reg address for the chip ID
#define OPR_MODE 0x3D				//reg address of the configuration mode register
#define CALIB_STAT_ADDR 0x35		//address of calibration status reg
#define GYROXADDR 0x14
#define ACCADDR 0x08				//address of Gyro X LSB, which is start of ALL the data we want to read
#define EULERADDR 0x1A
#define ACCOFFSET 0x55

#define CMD_READ_CHIP_ID 0x01
#define CMD_GOTO_NDOF_MODE 0x02
#define CMD_READ_CALIB_STATUS 0x03
#define CMD_READ_NDOF_DATA 0x04
#define CMD_READ_EULER_DATA 0x05
#define CMD_READ_ACC_DATA 0x08
#define CMD_READ_CALIB_DATA 0x06
#define CMD_READ_GYR_EULER_DATA 0x07

#define TESTPWM 4000	
	
	uint8_t command_sent=0;
	uint8_t process_reply_state=0;
	uint8_t BNChipID;
	uint8_t BNStatus;
	uint8_t BNCalibStatus=0;
	
	uint8_t BNOdata_pointer=0;
	uint8_t message_length=0;
	uint8_t BNOData[128];
	uint8_t BNOnewdataready=0;
	uint8_t eulerstatusflag=0;
	uint16_t samplecounter = 0;
	
	uint8_t eulerX_LSB,eulerX_MSB;
	uint8_t eulerY_LSB,eulerY_MSB;
	uint8_t eulerZ_LSB,eulerZ_MSB;
	
	int16_t eulerX, eulerY, eulerZ;
	int16_t gyrX, gyrY, gyrZ;
	int16_t accX, accY, accZ;
	int16_t Modifide_eulerX;
	int16_t last_euler;
	float32_t feulerX,feulerY,feulerZ = 0.0;
	float32_t feulerX_filtered,feulerY_filtered,feulerZ_filtered;
	float32_t euler_filter = 0;
	float32_t fgyrX,fgyrY,fgyrZ= 0.0;
	float32_t fgyrX_last, fgyrY_last, fgyrZ_last;
	float32_t fgyrX_filtered, fgyrY_filtered, fgyrZ_filtered;
	float32_t faccX, faccY, faccZ;
	float32_t roll,pitch,accel_vector; 
	float32_t filter_factor = 0.2;
	float32_t delta_gyrX, delta_gyrY, delta_gyrZ;
	float32_t debug_counter;
	float32_t debug_counter2;
	float32_t gyro_drift;
	int16_t i=0;
	
	float32_t pi = 3.14159265358979323;
	
	float32_t x =1;
	float32_t rusult;
	float32_t eulerZ_data[2];
	float32_t angle_per_sample;
	uint32_t useful;
	uint32_t total_full_osilations;
	float32_t total_angle;
	
	int16_t sample_number;
	float32_t frequency = 8.5;
	float32_t magX;
	float32_t magY;
	float32_t total_mag;
	float32_t angle;
	float32_t angle_current;
	float32_t last_angle;
	float32_t delta_angle;
	float32_t delta_angle_last;
	
	
	
	float64_t Input_Pitch;
	float64_t Input_Roll;
	float64_t Input_Throttle;
	float64_t Input_Yaw;
	
	
	
	
	
	
	float32_t cancelation;
	float32_t scalefactor = 0.6;
	float32_t delay = 2.7;
	
	
	
	uint8_t gyr_X_LSB, gyr_X_MSB;
	uint8_t gyr_Y_LSB, gyr_Y_MSB;
	uint8_t gyr_Z_LSB, gyr_Z_MSB;
	
	uint8_t acc_X_LSB, acc_X_MSB;
	uint8_t acc_Y_LSB, acc_Y_MSB;
	uint8_t acc_Z_LSB, acc_Z_MSB;
	
	float64_t motor1_out, motor2_out, motor3_out, motor4_out;		
	float64_t roll_factor, pitch_factor, yaw_factor;
	float64_t thrust_factor= 300;
	float64_t trim_roll = -7;
	float64_t trim_pitch;
	float32_t setpoint_yaw = 0.0;
	float32_t setpoint_pitch = 0.0;
	float32_t setpoint_roll = 0.0;
	float32_t setpoint_thrust = 0.0;
	char get_Input_command;
	float32_t requested_thrust = 0;
	float32_t requested_yaw = 0;
	float32_t requested_pitch = 0;
	float32_t requested_roll = 0;
	float64_t SHM_threshold;
	float32_t slew_factor = 0.5; // smooths the setpoint 

	uint8_t serial_time_out = 0;
	uint8_t serial_time_out_max = 50000;
	uint8_t max_safe_angle = 70;
	uint8_t stabilised_angle = 10 ;
	

	uint16_t Imax = 100000000;
	   
	
	float32_t setpoints_array [3][4] = {
		{0, 0, 0, 0},	//yaw angular setpoint, last angular setpoint, rate setpoint
		{0, 0, 0, 0},	//pitch angular setpoint, last angular setpoint, rate setpoint
		{0, 0, 0, 0},	//roll angular setpoint, last angular setpoint, rate setpoint
	};
	
	float64_t err [3][3] = {
		{0.0, 0.0, 0.0, 0.0},	//yaw angular error, sum_angular_error ,rate_err
		{0.0, 0.0, 0.0, 0.0},	//pitch angular error, sum_angular_error ,rate_err
		{0.0, 0.0, 0.0, 0.0 },
						//roll angular error, sum_angular_error ,rate_err
	};
	float64_t err_last;
	
	
	float64_t KPID [3][4] = {
		{0.0, 0.0, 0.1, 0.0000},	//yaw PIDD2  
		{0, 0.0000, 0.01, 0.0},	//pitch PIDD2" 0.43 0.0007 D=0.01
		{0, 0.0000, 0.01,0.0},
	//roll PID
	};
	
	
	float64_t Rate_Controller_KPID [3][3] = {
		{0.01, 0.00,0},	//pitch PID
		{0.01, 0.0000, 0.0},	//roll PID" 0.43 0.0007 D=0.01
		{0, 0.0000, 0.0}, //yaw
	//roll PID
	};
	
	float64_t P = 0.01;
	
	uint8_t goingToCrash;
	
	

	uint8_t hellostring[] = "Motion Robotics Cheap Flight Controller\r\n";

	uint8_t startingcalibrationstring[] = "Start doing the calibration...\r\n";

	uint8_t calibregisterstring[]="SYS GYR ACC MAG\r\n";
	
	uint8_t page_data[EEPROM_PAGE_SIZE];
	
	uint8_t hardwiredcalibration[22]={235
		,255
		,020
		,000
		,252
		,255
		,127
		,000
		,105
		,000
		,213
		,254
		,001
		,000
		,001
		,000
		,000
		,000
		,232
		,003
		,015
		,003};


	float cntrlRoll;
	float Remote_Roll;
	float cntrlPitch;
	float Remote_Pitch;
	float cntrlYaw;
	float Remote_Yaw;	
	float Remote_Throttle;
	
	float T[4];	


//struct PID_Struct Roll_PID;
//struct PID_Struct Pitch_PID;
//struct PID_Struct Yaw_PID;	
	//
	//
//void configure_eeprom(void)
//{
	///* Setup EEPROM emulator service */
	//
	//enum status_code error_code = eeprom_emulator_init();
//
	//if (error_code == STATUS_ERR_NO_MEMORY) {
		//while (true) {
			///* No EEPROM section has been set in the device's fuses */
			//printf("No EEPROM section has been set in the device's fuses!!\n\r");
			//delay_s(1);
		//}
		//} else if (error_code != STATUS_OK) {
		///* Erase the emulated EEPROM memory (assume it is unformatted or * irrecoverably corrupt) */
		//eeprom_emulator_erase_memory();
		//eeprom_emulator_init();
////		printf("I erased your EEPROM memory\n\r");
	//}
//
//}
//
//#if (SAMD || SAMR21)
//void SYSCTRL_Handler(void)
//{
	//if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		//SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33DET;
		//eeprom_emulator_commit_page_buffer();
	//}
//}
//#endif
//static void configure_bod(void)
//{
	//#if (SAMD || SAMR21)
	//struct bod_config config_bod33;
	//bod_get_config_defaults(&config_bod33);
	//config_bod33.action = BOD_ACTION_INTERRUPT;
	///* BOD33 threshold level is about 3.2V */
	//config_bod33.level = 48;
	//bod_set_config(BOD_BOD33, &config_bod33);
	//bod_enable(BOD_BOD33);
	//SYSCTRL->INTENSET.reg = SYSCTRL_INTENCLR_BOD33DET;
	//system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
	//#endif
//
//}
//
void harmonic_cancelation (void)
{
	
	angle_per_sample =(0.02*pi)*frequency;
	useful = (1 * 2*pi)/angle_per_sample;
	eulerZ_data[sample_number] = feulerZ;
	
	for(i=0;i<useful;i++)
	{
		magX = magX + ((sin(i*angle_per_sample))* eulerZ_data[i]);
		magY = magY + ((cos(i*angle_per_sample))* eulerZ_data[i]);
	}
	
	rusult = (sqrt((magX*magX) + (magY*magY)));
	
	if (asin(magX/rusult) > 0)
	{
		angle = acos(magY/rusult);
	}
	else
	{
		angle = 2*pi - acos(magY/rusult);
	}
	
	if (rusult>50)
	{
		angle_current = sample_number*angle_per_sample;
		delta_angle = angle - angle_current;
		cancelation = - cos(delta_angle+delay)*scalefactor*rusult;
	}
	else
	{
		
		cancelation = 0;
	}
	//delta_angle = angle - last_angle;
	//if ( delta_angle< -1* pi)
	//{
	//	delta_angle = 0;
	//}
	//if ( delta_angle> 1* pi)
//	{
	//	delta_angle = 0;
	//}
	//printf("%f,%f, %f, %f, %f , %f,%f, \r\n", rusult,angle_current, angle,feulerZ,scalefactor, delay, cancelation);
	printf("%f,%f\r\n",feulerZ, fgyrZ);
	magX = 0;
	magY = 0;
	rusult = 0;
	last_angle = angle;
	sample_number++;
	//delta_angle_last = delta_angle;

	if (sample_number == useful)
	{
		sample_number = 0;
	}
	
	 // number of measurements for 1 cycle

}
void drive_motor(uint8_t motornumber, uint16_t powerlevel)
{
	//expecting a motor number 1 to 4
	//expecting a powerlevel between 0 and 8000

	float64_t pwmvalue;
	
	pwmvalue=powerlevel*7.95;
	if(pwmvalue > 7950) pwmvalue=7950;

	tcc_set_compare_value(&tcc_zero_instance,motornumber,7890+pwmvalue);
	
	
}
void fft (void)
{
	for(frequency=0.1;frequency<9.9;frequency = frequency +0.1)
	{
		angle_per_sample =(0.02*pi)*frequency;
		total_angle = sample_number * angle_per_sample;
		total_full_osilations = (total_angle/2*pi) - 1;
		useful = (total_full_osilations * 2*pi)/angle_per_sample;
		magX = 0;
		magY = 0;
		rusult = 0;
		for(i=0;i<useful;i++) 
		{
			magX = magX + ((sin(i*angle_per_sample))* eulerZ_data[i]);
			magY = magY + ((cos(i*angle_per_sample))* eulerZ_data[i]);
			
			
		}
		rusult = sqrt((magX*magX) + (magY*magY));
		printf("%f,%f  \r\n", frequency,rusult);
	}
}

void dont (void)
{
	drive_motor(0,0);
	drive_motor(1,0);
	drive_motor(2,0);
	drive_motor(3,0);
	fft();
	
	while (1){}
}

void safety_status (void)
{
	//if (feulerY > max_safe_angle || feulerY < -max_safe_angle || feulerZ > max_safe_angle || feulerZ < -max_safe_angle )
		//{
		//	goingToCrash = 1;
		//	printf("max safe angle\r\n");
		//}
	
	
	if (goingToCrash == 1)
	{
		printf("%f,%f,%f,%f,%f\r\n",feulerZ, fgyrZ, debug_counter, KPID[1][2], KPID[1][3]);
		dont();
	}
} 

void update_setpoints (void)
{
	//setpoint_pitch = slew_factor * (requested_pitch - setpoint_pitch) + setpoint_pitch ;
	//setpoint_roll =  slew_factor * (requested_roll - setpoint_roll) + setpoint_roll   ;
}

void get_control_inputs (void)
{
		
			if(newkeydata!=0)
			{
				usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);

				switch(newkeydata)
				{
					
					case '.':
					delay = delay+ 0.01*pi;
					break;
					
					case ',':
					delay = delay- 0.01*pi;
					break;
					
				case 'b':
				scalefactor = scalefactor + 0.01;
				break;

				case 'v':
				scalefactor = scalefactor - 0.01;
				break;
					
					case 't':
					trim_pitch--;  
					break;

					case 'f':
					trim_pitch++;  
					break;
					
					case 'e':
					trim_roll--;
					break;

					case 'r':
					trim_roll++;
					break;
					
					case 'w':
					setpoint_pitch = -stabilised_angle  ;
					break;

					case 'a':
					setpoint_roll = stabilised_angle  ;
					break;
					
					case 's':
					setpoint_pitch = stabilised_angle  ;
					break;

					case 'd':
					setpoint_roll = -stabilised_angle  ;
					break;
					
					case 'z':
					thrust_factor = thrust_factor - 5;
					break;
					
					case 'x':
					thrust_factor = thrust_factor + 5;
					break;
					
					case'q':
					goingToCrash = 1;
					break;
					
					case 'p':
					KPID[1][0] = KPID[1][0] + 0.1;
					KPID[2][0] = KPID[2][0] + 0.1;
				
					break;
							
					case 'l':
					KPID[1][0] = KPID[1][0] - 0.1;
					KPID[2][0] = KPID[2][0] - 0.1;
					
					break;
					
					
					case 'i':
					KPID[1][1] = KPID[1][1] + 0.0001;
					KPID[2][1] = KPID[2][1] + 0.0001;
					break;
					
					case 'j':
					KPID[1][1] = KPID[1][1] - 0.0001;
					KPID[2][1] = KPID[2][1] - 0.0001;
					break;
					
					case 'm':
					KPID[1][2] = KPID[1][2] + 0.001;
					KPID[2][2] = KPID[2][2] + 0.001;
					break;
					
					case 'n':
					KPID[1][2] = KPID[1][2] - 0.001;
					KPID[2][2] = KPID[2][2] - 0.001;
					break;
					
					case 'y':
					KPID[1][3] = KPID[1][3] + 0.01;
					KPID[2][3] = KPID[2][3] + 0.01;
					break;
					
					case 'g':
					KPID[1][3] = KPID[1][3] - 0.01;
					KPID[2][3] = KPID[2][3] - 0.01;
					break;
					
					
					
				}
				newkeydata=0;
				serial_time_out = 0;
			}
			else 
			{
				serial_time_out++;
			}
			
			if (serial_time_out > 30)
				{
					setpoint_pitch = 0.0  ;
					setpoint_roll = 0.0  ;
				}
		
			
	//update_setpoints();

}
void rate_profile (void)
{
	
	//int16_t 
	if (Channel1 > 1000 && Channel1 < 2000)
	{
	Input_Pitch = (float64_t)Channel1 - 1500;
	Input_Pitch = 0.000003*((Input_Pitch)*(Input_Pitch)*(Input_Pitch))+0.2*(Input_Pitch); //cubic rate profile aprox 500dps
	Input_Pitch = 10*Input_Pitch;	
	}
	if (Channel2 > 1000 && Channel2 < 2000)
	{
		Input_Roll = ((float64_t)Channel2 - 1500);
		Input_Roll = 0.000003*((Input_Roll)*(Input_Roll)*(Input_Roll))+0.2*(Input_Roll); //cubic rate profile aprox 500dps
		Input_Roll = 10*Input_Roll;	
	}
	if (Channel3 > 1000 && Channel3 < 2000)
	{
		Input_Throttle = (float64_t)Channel3 - 1000;
		thrust_factor = 0.8*(Input_Throttle);
	}
	if (Channel3 > 1000 && Channel3 < 2000)
	{
		Input_Yaw = ((float64_t)Channel4 - 1500);
		Input_Yaw = 0.000002*((Input_Yaw)*(Input_Yaw)*(Input_Yaw))+0.2*(Input_Yaw); //cubic rate profile aprox 500dps
		Input_Yaw = 8*-Input_Yaw;
	}
//Input_Pitch=0;
//Input_Roll= 0;
Input_Throttle = 300;
}


void rate_controller2.0 (void)
{
	//uint8_t p = 0;
	//uint8_t I = 1;
	//uint8_t D = 2;
	//uint8_t R = 1;
	//uint8_t Y = 2;
	//float64_t PID_Controller[3][3];
	
	//PID_Controller[P][D] = 
	//PID_Controller[p][p] = Rate_Controller_KPID[p][p]* (Input_Pitch - fgyrY);
	
	
	
}
void calculate_pid_output (void) //PID loop
{
i = 0;
	setpoints_array[i][1] = setpoints_array[i][0];
	setpoints_array[i][0] = setpoint_yaw;
	setpoints_array[i][2] = setpoints_array[i][0] - setpoints_array[i][1];
err [i][0]= (setpoints_array[i][0] - feulerX);		//p
if (err[i][0] > 180)
{
	err [i][0]= -360 + err [i][0];
}
if (err[i][0] < -180)
{
	err [i][0]= +360 + err [i][0];
}
err [i][1]= (err[i][1] + err[i][0]);		//I
if (err[i][1] > Imax)
{
	err[i][1] = Imax;
}
if (err[i][1] < -Imax)
{
	err[i][1] = -Imax;
}
err [i][2]= (fgyrX - Input_Yaw);		//D



i = 1;	
setpoints_array[i][1] = setpoints_array[i][0];
setpoints_array[i][0] = setpoint_pitch + trim_pitch;
setpoints_array[i][2] = setpoints_array[i][0] - setpoints_array[i][1];
												//PITCH
err [i][0]= (setpoints_array[i][0] - feulerY);		//p
err [i][1]= (err[i][1] + err[i][0]);				//I
if (err[i][1] > Imax)
{
	err[i][1] = Imax;
}
if (err[i][1] < -Imax)
{
	err[i][1] = -Imax;
}
err_last = err [i][2];
err [i][2]= (fgyrY_filtered - Input_Pitch);		//D

err [i][3]= (err [i][2] - err_last );		//D2





i=2;
setpoints_array[i][1] = setpoints_array[i][0];								//P last
setpoints_array[i][0] = setpoint_roll + trim_roll ;							//P
setpoints_array[i][2] = setpoints_array[i][0] - setpoints_array[i][1];		// delta setpoint
err [i][0]= (setpoints_array[i][0] - feulerZ);			//P
err [i][1]= (err[i][1] + err[i][0]);						//I
if (err[i][1] > Imax)
{
	err[i][1] = Imax;
}
if (err[i][1] < -Imax)
{
	err[i][1] = -Imax;
}
;
err_last = err [i][2];
err [i][2]= (fgyrZ_filtered - Input_Roll );				//D
err [i][3]= (err [i][2] - err_last );							//D2


i = 0;
yaw_factor = (err[i][0]*KPID[i][0])+(err[i][1]*KPID[i][1])+(err[i][2]*KPID[i][2]);
i = 1;
pitch_factor = (err[i][0]*KPID[i][0])+(err[i][1]*KPID[i][1])+(err[i][2]*KPID[i][2])+ (err[i][3]*KPID[i][3]);
i= 2;
roll_factor = (err[i][0]*KPID[i][0])+(err[i][1]*KPID[i][1])+(err[i][2]*KPID[i][2])+ (err[i][3]*KPID[i][3]) ;



//pitch_factor =
//roll_factor =
//gyrZ = 10*gyrZ;
//printf("%f, %f,%f, %f, %f, %f\r\n",KPID[1][0], KPID[1][1], KPID[1][2], KPID[1][3], filter_factor,euler_filter);
//printf("%f\r\n", roll_factor);


			//printf("%d \n\r",setpoints_array[1][1]);
			 
	//printf("%f ,%f ,%f  \n\r ", setpoints_array[1][1], setpoints_array[1][2], setpoints_array[1][3]);
	

//printf(" %f \r\n", err[3][2]);
//printf("%f ,%f ,%f  \r\n", err[i][1], err[i][2], err[i][3]);


}			//PidS


static void configure_usart_cdc(void)
{

	struct usart_config config_cdc;
	usart_get_config_defaults(&config_cdc);
	config_cdc.baudrate	 = 57600;
	config_cdc.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_cdc.pinmux_pad0 = PINMUX_PA06D_SERCOM0_PAD2;
	config_cdc.pinmux_pad1 = PINMUX_PA07D_SERCOM0_PAD3;
	config_cdc.pinmux_pad2 = PINMUX_UNUSED;
	config_cdc.pinmux_pad3 = PINMUX_UNUSED;
	stdio_serial_init(&cdc_instance, SERCOM0, &config_cdc);
	usart_enable(&cdc_instance);
}
	
void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate	 = 57600;
//	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
//	config_usart.pinmux_pad0 = PINMUX_PA10C_SERCOM0_PAD2;
//	config_usart.pinmux_pad1 = PINMUX_PA11C_SERCOM0_PAD3;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_PA06D_SERCOM0_PAD2;
	config_usart.pinmux_pad1 = PINMUX_PA07D_SERCOM0_PAD3;

	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
	while (usart_init(&cdc_instance, SERCOM0, &config_usart) != STATUS_OK)
	{
		
}
	usart_enable(&cdc_instance);
}



//! [cdc_setup]
void usart_read_callback(struct usart_module *const usart_module)
{
	newkeydata=rx_buffer[0];
	
	usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);

}

void usart_write_callback(struct usart_module *const usart_module)
{
//	port_pin_toggle_output_level(PIN_PA17);
}
void configure_usart_callbacks(void)
{
	//! [setup_register_callbacks]
	usart_register_callback(&cdc_instance,
	usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&cdc_instance,
	usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_register_callbacks]

	//! [setup_enable_callbacks]
	usart_enable_callback(&cdc_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&cdc_instance, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_enable_callbacks]
}


void configure_usart_MATEK(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate	 = 115200;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_PA10D_SERCOM2_PAD2;
	config_usart.pinmux_pad1 = PINMUX_PA11D_SERCOM2_PAD3;
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
	//Channel1_lsb = 3;
	while (usart_init(&cdc_instance_MATEK, SERCOM2, &config_usart) != STATUS_OK)
	{
		
	}
	usart_enable(&cdc_instance_MATEK);
}



//! [cdc_setup]
void usart_read_callback_MATEK(struct usart_module *const usart_module)
{
//	tx_buffer[0]=rx_buffer[0];
	//usart_write_buffer_job(&cdc_instance_MATEK,(uint8_t *)tx_buffer, 1);
//Channel1_lsb = 3;
	
				switch(read_flysky_state)
				 {
					 
					 case 0:
					 if (MATEK_rx_buffer[0] == 0x20)
					 {
						read_flysky_state++;
						usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					 }
					 else
					 {
						 usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);  //header
					 }
					 
					  case 1:
					  if (MATEK_rx_buffer[0] == 0x40)
					  {
						  read_flysky_state++;
						  read_flysky_state2=0;
						  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  }
					 else
					 {
						 read_flysky_state = 0;
				     	  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);//header
					 }
					 
					 case 2:
					 read_flysky_state++;
					// Channel1_msb=MATEK_rx_buffer[0];
					 usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);  //leave first byte
					 break;
					 
					 case 3:
					 read_flysky_state++;
					 Channel1_lsb=MATEK_rx_buffer[0];
					 usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					 break;
					 
					  case 4:
					  read_flysky_state++;
					  Channel1_msb=MATEK_rx_buffer[0];
					  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  break;
					  
					  case 5:
				 read_flysky_state++;
					  Channel2_lsb=MATEK_rx_buffer[0];
					  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  break;
					  
					  case 6:
				 read_flysky_state++;
					  Channel2_msb=MATEK_rx_buffer[0];
					  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  break;
					  
					  case 7:
					  read_flysky_state++;
					  Channel3_lsb=MATEK_rx_buffer[0];
					  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  break;
					  case 8:
					  read_flysky_state++;
					  Channel3_msb=MATEK_rx_buffer[0];
					  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  break;
					  case 9:
					  read_flysky_state++;
					  Channel4_lsb=MATEK_rx_buffer[0];
					  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  break;
					  case 10:
					  read_flysky_state++;
					  Channel4_msb=MATEK_rx_buffer[0];
					  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
					  break;
					  
					  case 11:
					  read_flysky_state=11;
					  read_flysky_state2++;
					  if (read_flysky_state2 < 18)
					  {
						  usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);  //leave first byte
					  }
					  else
					  {
						read_flysky_state = 0;					
					  }
					  break;
					
				 }
			
	
	//usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
//Channel1_lsb = 3;
}


void get_flysky_input(void)
{
//	debug_counter2 = 3.0;
	//printf("i am here \n\r ");
	Channel1 = Channel1_msb*256;
	Channel1+= Channel1_lsb;
	
	Channel2 = Channel2_msb*256;
	Channel2+= Channel2_lsb;
	
	Channel3 = Channel3_msb*256;
	Channel3+= Channel3_lsb;
	
	Channel4 = Channel4_msb*256;
	Channel4+= Channel4_lsb;
	
	//debug_counter2 = 3.0;
	usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);
	//printf("%i , %i \n\r", Channel1,Channel2);
}


//void usart_write_callback_MATEK(struct usart_module *const usart_module)
//{
	//	port_pin_toggle_output_level(PIN_PA17);
//}
void configure_usart_callbacks_MATEK(void)
{
	//! [setup_register_callbacks]
	//usart_register_callback(&cdc_instance_MATEK,
	//usart_write_callback_MATEK, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&cdc_instance_MATEK,
	usart_read_callback_MATEK, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_register_callbacks]

	//! [setup_enable_callbacks]
	//usart_enable_callback(&cdc_instance_MATEK, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&cdc_instance_MATEK, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_enable_callbacks]
}



void configure_usart_BNO005(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate	 = 115200;
	config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_usart.pinmux_pad0 = PINMUX_PB08D_SERCOM4_PAD0;
	config_usart.pinmux_pad1 = PINMUX_PB09D_SERCOM4_PAD1;
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
	while (usart_init(&cdc_instance_BNO005, SERCOM4, &config_usart) != STATUS_OK)
	{
		
	}
	usart_enable(&cdc_instance_BNO005);
}



//! [cdc_setup]
void usart_read_BNO005_callback(struct usart_module *const usart_module) //read BNO055
{
		switch(command_sent)
		{

			case CMD_READ_EULER_DATA:
				switch(process_reply_state)
				{
					case 0x00:		// then first byte being returned
					if(BNO005_rx_buffer[0]==0xEE)	//check for error
					{
						process_reply_state=0xEE;
						usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
						break;
					}
					if(BNO005_rx_buffer[0]==0xBB)	//good read
					{
						process_reply_state=1;
						usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
						break;
					}
					while(1)
					{
						port_pin_toggle_output_level(PIN_PA17);
						delay_ms(500);
						
					}
					
				
					case 0x01:
					process_reply_state=2;    //just let the length byte go
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer); /////gyro data
					break;
					case 0x02:
					process_reply_state=0x03;
					gyr_X_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x03:
					process_reply_state=0x04;
					gyr_X_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x04:
					process_reply_state=0x05;
					gyr_Y_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x05:
					process_reply_state=0x06;
					gyr_Y_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x06:
					process_reply_state=0x07;
					gyr_Z_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x07:
					process_reply_state=0x09;
					gyr_Z_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					
					
					
					
					case 0x09:
					process_reply_state=0x0A;
					acc_X_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0A:
					process_reply_state=0x0B;
					acc_X_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0B:
					process_reply_state=0x0C;
					acc_Y_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0C:
					process_reply_state=0x0D;
					acc_Y_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0D:
					process_reply_state=0x0E;
					acc_Z_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0E:
					process_reply_state=0;
					acc_Z_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
				
					break;
					
					
				}
				break;
					
					//
					//case 0x08:
					//process_reply_state=0x09;
					//eulerX_LSB=BNO005_rx_buffer[0];
					//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					//break;
					//case 0x09:
					//process_reply_state=0x0A;
					//eulerX_MSB=BNO005_rx_buffer[0];
					//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					//break;
					//case 0x0A:
					//process_reply_state=0x0B;
					//eulerY_LSB=BNO005_rx_buffer[0];
					//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					//break;
					//case 0x0B:
					//process_reply_state=0x0C;
					//eulerY_MSB=BNO005_rx_buffer[0];
					//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					//break;
					//case 0x0C:
					//process_reply_state=0x0D;
					//eulerZ_LSB=BNO005_rx_buffer[0];
					//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					//break;
					//case 0x0D:
					//eulerZ_MSB=BNO005_rx_buffer[0];
					//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					//command_sent=0;
					//process_reply_state=0;
					//eulerstatusflag = 1;
					//break;
					//case 0xEE:
					//BNStatus=BNO005_rx_buffer[0];
					//process_reply_state=0;
					//command_sent=0;
					//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					//
					//break;
					
					
				
				
				case CMD_READ_ACC_DATA:
				switch(process_reply_state)
				{
					//case 0x00:		// then first byte being returned
				//	if(BNO005_rx_buffer[0]==0xEE)	//check for error
				//	{
					//	process_reply_state=0xEE;
					//	usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
				//		break;
				//	}
					//if(BNO005_rx_buffer[0]==0xBB)	//good read
					//{
					//	process_reply_state=1;
						//usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
						//break;
				//	}
					//while(1)
				//	{
					//	port_pin_toggle_output_level(PIN_PA17);
						//delay_ms(500);
						
				//	}
					case 0x00:
					process_reply_state=1;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					
					case 0x01:
					process_reply_state=2;    //just let the length byte go
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer); /////gyro data
					break;
					case 0x02:
					process_reply_state=0x03;
					acc_X_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x03:
					process_reply_state=0x04;
					acc_X_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x04:
					process_reply_state=0x05;
					acc_Y_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x05:
					process_reply_state=0x06;
					acc_Y_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x06:
					process_reply_state=0x07;
					acc_Z_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x07:
					process_reply_state=0x08;
					acc_Z_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					
					
					case 0x08:
					process_reply_state=0x09;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x09:
					process_reply_state=0x0A;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0A:
					process_reply_state=0x0B;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0B:
					process_reply_state=0x0C;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x0C:
					process_reply_state=0x0D;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
			
			
			
			
					
				
					
					break;
					case 0x0D:
					process_reply_state=0x0F;    //just let the length byte go
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer); /////gyro data
					break;
					case 0x0F:
					process_reply_state=0x10;
					gyr_X_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x10:
					process_reply_state=0x11;
					gyr_X_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x11:
					process_reply_state=0x12;
					gyr_Y_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x12:
					process_reply_state=0x13;
					gyr_Y_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x13:
					process_reply_state=0x14;
					gyr_Z_LSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x14:
					process_reply_state=0;
					gyr_Z_MSB=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					
					
				}
				
				break;
			
			case CMD_READ_NDOF_DATA:
			switch(process_reply_state)
			{
				case 0x00:		// then first byte being returned
				if(BNO005_rx_buffer[0]==0xEE)	//check for error
				{
					process_reply_state=0xEE;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
				}
				if(BNO005_rx_buffer[0]==0xBB)	//good read
				{
					process_reply_state=1;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
				}
				while(1)
				{
					port_pin_toggle_output_level(PIN_PA17);
					delay_ms(500);
				}
				case 0x01:
					process_reply_state=2;    
					message_length=BNO005_rx_buffer[0];	//load up the number of bytes that will follow, should be 20
					BNOdata_pointer=0;	//set pointer to start of array
					
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					
				break;
				case 0x02:
					BNOData[BNOdata_pointer]=BNO005_rx_buffer[0];
					BNOdata_pointer++;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					if(BNOdata_pointer==message_length)
					{
						command_sent=0;
						process_reply_state=0;
						message_length=0;
						BNOnewdataready=1;
					}
				break;
				case 0xEE:
					BNStatus=BNO005_rx_buffer[0];
					process_reply_state=0;
					command_sent=0;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
				break;
				
				
			}
			
			break;			

			case CMD_READ_CALIB_DATA:
			switch(process_reply_state)
			{
				case 0x00:		// then first byte being returned
				if(BNO005_rx_buffer[0]==0xEE)	//check for error
				{
					process_reply_state=0xEE;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
				}
				if(BNO005_rx_buffer[0]==0xBB)	//good read
				{
					process_reply_state=1;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
				}
				while(1)
				{
					port_pin_toggle_output_level(PIN_PA17);
					delay_ms(500);
				}
				case 0x01:
				process_reply_state=2;
				message_length=BNO005_rx_buffer[0];	//load up the number of bytes that will follow, should be 22
				BNOdata_pointer=0;	//set pointer to start of array
				
				usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
				
				break;
				case 0x02:
				BNOData[BNOdata_pointer]=BNO005_rx_buffer[0];
				BNOdata_pointer++;
				usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
				if(BNOdata_pointer==message_length)
				{
					command_sent=0;
					process_reply_state=0;
					message_length=0;
					BNOnewdataready=1;
				}
				break;
				case 0xEE:
				BNStatus=BNO005_rx_buffer[0];
				process_reply_state=0;
				command_sent=0;
				usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
				break;
				
				
			}
			
			break;

			
			case CMD_READ_CHIP_ID:
				switch(process_reply_state)  
				{
					case 0x00:		// then first byte being returned
						if(BNO005_rx_buffer[0]==0xEE)	//check for error
						{
							process_reply_state=0xEE;
							usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
							break;
						}
						if(BNO005_rx_buffer[0]==0xBB)	//good read
						{	
							process_reply_state=1;
							usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
							break;							
						}
						while(1)
						{
							port_pin_toggle_output_level(PIN_PA17);
							delay_ms(500);
						}
					case 0x01:
							process_reply_state=2;    //just let the length byte go
							usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
							break;
					case 0x02:
							BNChipID=BNO005_rx_buffer[0];
							usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
							command_sent=0;
							process_reply_state=0;
							break;	
					case 0xEE:
							BNStatus=BNO005_rx_buffer[0];
							process_reply_state=0;
							command_sent=0;
							usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
							break;
					
					
				}
				
			break;

			case CMD_READ_CALIB_STATUS:
			
				switch(process_reply_state)
				{
					case 0x00:		// then first byte being returned
					if(BNO005_rx_buffer[0]==0xEE)	//check for error
					{
						process_reply_state=0xEE;
						usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
						break;
					}
					if(BNO005_rx_buffer[0]==0xBB)	//good read
					{
						process_reply_state=1;
						usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
						break;
					}
					while(1)
					{
						port_pin_toggle_output_level(PIN_PA17);
						delay_ms(500);
					}
					case 0x01:
					process_reply_state=2;    //just let the length byte go
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
					case 0x02:
					BNCalibStatus=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					command_sent=0;
					process_reply_state=0;
					break;
					case 0xEE:
					BNStatus=BNO005_rx_buffer[0];
					process_reply_state=0;
					command_sent=0;
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					break;
				
				
				}
			
			break;
			
			case CMD_GOTO_NDOF_MODE:
				switch(process_reply_state)
				{
					case 0x00:		// then first byte being returned
					if(BNO005_rx_buffer[0]==0xEE)	//check for response
					{
						process_reply_state=0x02;
						usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
						break;
					}
					while(1)
					{
						port_pin_toggle_output_level(PIN_PA17);
						delay_ms(500);
					}
					case 0x02:
					BNStatus=BNO005_rx_buffer[0];
					usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
					command_sent=0;
					process_reply_state=0;
					
					if(BNStatus!=1)
					{
						while(1)
						{
							port_pin_toggle_output_level(PIN_PA17);
							delay_ms(500);
						}
						
					}
					
					break;
				}

			
			default:
				usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
			
			
		}

}

void usart_write_BNO005_callback(struct usart_module *const usart_module)
{
	//	port_pin_toggle_output_level(PIN_PA17);
}
void configure_BNO005_usart_callbacks(void)
{
	//! [setup_register_callbacks]
	usart_register_callback(&cdc_instance_BNO005,	usart_write_BNO005_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&cdc_instance_BNO005,	usart_read_BNO005_callback, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_register_callbacks]

	//! [setup_enable_callbacks]
	usart_enable_callback(&cdc_instance_BNO005, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&cdc_instance_BNO005, USART_CALLBACK_BUFFER_RECEIVED);
	//! [setup_enable_callbacks]
}

void configu_eic(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin = PIN_PA20A_EIC_EXTINT4;   //PIN_PA09A_EIC_EXTINT9;
	config_extint_chan.gpio_pin_mux =MUX_PA20A_EIC_EXTINT4;  //MUX_PA09A_EIC_EXTINT9;
	config_extint_chan.gpio_pin_pull = EXTINT_PULL_NONE;
	config_extint_chan.detection_criteria = EXTINT_DETECT_HIGH;
	extint_chan_set_config(4, &config_extint_chan);
	struct extint_events config_events = {
		.generate_event_on_detect[4] = true
	};
	extint_enable_events(&config_events);

}

void configure_tcc_zero(void)
{


	struct tcc_config config_tcc;
	tcc_get_config_defaults(&config_tcc, TCC0); //was TCC0
	config_tcc.counter.clock_source = GCLK_GENERATOR_0;
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;

	
	config_tcc.counter.period = 0x8FF0;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	
	
	
	config_tcc.wave.wave_polarity[0]=0;
	config_tcc.wave.wave_polarity[1]=0;


	config_tcc.compare.match[0] = 0;
	config_tcc.pins.enable_wave_out_pin[0] = true;
	config_tcc.pins.wave_out_pin[0]        = PIN_PA08E_TCC0_WO0;
	config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA08E_TCC0_WO0;

	config_tcc.compare.match[1] = 0;
	config_tcc.pins.enable_wave_out_pin[1] = true;
	config_tcc.pins.wave_out_pin[1]        = PIN_PA09E_TCC0_WO1;
	config_tcc.pins.wave_out_pin_mux[1]    = MUX_PA09E_TCC0_WO1;

	config_tcc.compare.match[2] = 0;
	config_tcc.pins.enable_wave_out_pin[2] = true;
	config_tcc.pins.wave_out_pin[2]        = PIN_PA18F_TCC0_WO2;
	config_tcc.pins.wave_out_pin_mux[2]    = MUX_PA18F_TCC0_WO2;


	config_tcc.compare.match[3] = 0;
	config_tcc.pins.enable_wave_out_pin[3] = true;
	config_tcc.pins.wave_out_pin[3]        = PIN_PA19F_TCC0_WO3;
	config_tcc.pins.wave_out_pin_mux[3]    = MUX_PA19F_TCC0_WO3;

	tcc_init(&tcc_zero_instance, TCC0, &config_tcc);
	tcc_enable(&tcc_zero_instance);
}

void configure_evsys(void)
{
	struct events_config config;
	events_get_config_defaults(&config);
	config.clock_source = GCLK_GENERATOR_0;
	config.generator    = EVSYS_ID_GEN_EIC_EXTINT_4;
	config.path         = EVENTS_PATH_ASYNCHRONOUS;
	config.edge_detect  = EVENTS_EDGE_DETECT_BOTH;
	events_allocate(&event_resources, &config);
	events_attach_user(&event_resources, EVSYS_ID_USER_TCC1_EV_1);
}


void printsigneddata(char * header, int16_t printint)
{
	int16_t  partial,actual;

	messagestring[0]=' ';
	
	if(printint <0)
	{
		messagestring[1]='-';
		actual=0-printint;
	}
	else
	{
		messagestring[1]='+';
		actual=printint;
	}
	
	partial=actual;
	
	
	actual=partial/10000;
	messagestring[2]='0'+ actual;
	partial= partial - actual*10000;
	
	actual=partial/1000;
	messagestring[3]='0'+ actual;
	partial= partial - actual*1000;
	
	actual=partial/100;
	messagestring[4]='0'+ actual;
	partial= partial - actual*100;
	
	actual=partial/10;
	messagestring[5]='0'+ actual;
	partial= partial - actual*10;
	
	actual=partial;
	messagestring[6]='0'+ actual;
	
	usart_write_buffer_wait(&cdc_instance, header, 8);
	usart_write_buffer_wait(&cdc_instance, messagestring, sizeof(messagestring));
	
	
}

void printdata(char * header, uint16_t printint)
{
	uint16_t  partial,actual;

				actual= printint;
				
				partial=actual; 
				
				
				actual=partial/10000;
				messagestring[2]='0'+ actual;
				partial= partial - actual*10000;
				
				actual=partial/1000;
				messagestring[3]='0'+ actual;
				partial= partial - actual*1000;
				
				actual=partial/100;
				messagestring[4]='0'+ actual;
				partial= partial - actual*100;
				
				actual=partial/10;
				messagestring[5]='0'+ actual;
				partial= partial - actual*10;
				
				actual=partial;
				messagestring[6]='0'+ actual;
				
				usart_write_buffer_wait(&cdc_instance, header, 5);
				usart_write_buffer_wait(&cdc_instance, messagestring, sizeof(messagestring));

}

void printbinary(uint8_t binval)
{

	if(binval & 0b10000000) binarystring[2]='1';
	else binarystring[2]='0';
	if(binval & 0b01000000) binarystring[3]='1';
	else binarystring[3]='0';
	if(binval & 0b00100000) binarystring[4]='1';
	else binarystring[4]='0';
	if(binval & 0b00010000) binarystring[5]='1';
	else binarystring[5]='0';
	if(binval & 0b00001000) binarystring[6]='1';
	else binarystring[6]='0';
	if(binval & 0b00000100) binarystring[7]='1';
	else binarystring[7]='0';
	if(binval & 0b00000010) binarystring[8]='1';
	else binarystring[8]='0';
	if(binval & 0b10000001) binarystring[9]='1';
	else binarystring[9]='0';
	
	usart_write_buffer_wait(&cdc_instance, binarystring, sizeof(binarystring));

}

void configure_tcc_one(void)
{
	struct tcc_config config_tcc;
	tcc_get_config_defaults(&config_tcc, TCC1);
	config_tcc.counter.clock_source         = GCLK_GENERATOR_0;
	config_tcc.counter.clock_prescaler      = TCC_CLOCK_PRESCALER_DIV1;
	config_tcc.compare.channel_function[0]  = TCC_CHANNEL_FUNCTION_CAPTURE;
	config_tcc.compare.channel_function[1]  = TCC_CHANNEL_FUNCTION_CAPTURE;
	//	config_tcc.compare.channel_function[2]  = TCC_CHANNEL_FUNCTION_CAPTURE;
	//	config_tcc.compare.channel_function[3]  = TCC_CHANNEL_FUNCTION_CAPTURE;
	config_tcc.double_buffering_enabled     = false;
	tcc_init(&tcc_instances, TCC1, &config_tcc);

	struct tcc_events events_tcc = {
		.input_config[0].modify_action      = false,
		.input_config[1].modify_action      = true,
		.on_input_event_perform_action[1]   = true,
		.input_config[1].action             = TCC_EVENT1_ACTION_PULSE_WIDTH_PERIOD_CAPTURE,
	};
	tcc_enable_events(&tcc_instances, &events_tcc);
	tcc_enable(&tcc_instances);

}

void null_motor(uint8_t motornumber)
{
	//expecting a motor number 1 to 4
	//set PWM pulse to .9ms

	tcc_set_compare_value(&tcc_zero_instance,motornumber,7950);
	
	
}




#define CHIP_ID 0x00

void read_chip_ID(void)
{
		uint8_t command[4];
		
		command[0]=0xAA;	//header
		command[1]=0x01;	//read command
		command[2]=CHIP_ID;	//chip id register address
		command[3]=1;		//number of data bytes in message
		
		process_reply_state=0;
		command_sent=CMD_READ_CHIP_ID;
		usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
		usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
		
		
}
void set_to_CONFIG_mode(void)
{
	uint8_t command[5];
	
	command[0]=0xAA;	//header
	command[1]=0x00;	//write command
	command[2]=OPR_MODE;	//OPR_MODE register address
	command[3]=1;		//number of data bytes in message
	command[4]=0x00;	//select config mode
	
	process_reply_state=0;
	command_sent=CMD_GOTO_NDOF_MODE;
	usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
	usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
	
}


void set_to_NDOF_mode(void)
{
		uint8_t command[5];
		
		command[0]=0xAA;	//header
		command[1]=0x00;	//write command
		command[2]=OPR_MODE;	//OPR_MODE register address
		command[3]=1;		//number of data bytes in message
		command[4]=0x05;	//select NDOF mode
		
		process_reply_state=0;
		command_sent=CMD_GOTO_NDOF_MODE;
		usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
		usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
	
}

void read_calib_status(void)
{
		uint8_t command[4];
		
		command[0]=0xAA;	//header
		command[1]=0x01;	//read command
		command[2]=CALIB_STAT_ADDR;	//chip id register address
		command[3]=1;		//number of data bytes in message
		
		process_reply_state=0;
		command_sent=CMD_READ_CALIB_STATUS;
		usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
		usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
		
			
	
}

void read_all_BNO_data(void)
{
		uint8_t command[4];
		
		for(BNOdata_pointer=0;BNOdata_pointer<128;BNOdata_pointer++)
		{
			BNOData[BNOdata_pointer]=0;		//clear all data
		}
		
		command[0]=0xAA;	//header
		command[1]=0x01;	//read command
		command[2]=GYROXADDR;	//Gyro data X LSB register address
		command[3]=20;		//number of data bytes in message
		
		process_reply_state=0;
		command_sent=CMD_READ_NDOF_DATA;
		BNOnewdataready=0;
		usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
		usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
	
}

void read_acc_gyr_data (void)
{
	uint8_t command[4];
	
	for(BNOdata_pointer=0;BNOdata_pointer<18;BNOdata_pointer++)
	{
		BNOData[BNOdata_pointer]=0;		//clear all data
	}
	
	command[0]=0xAA;	//header
	command[1]=0x01;	//read command
	command[2]=ACCADDR;	//EULER Address start
	command[3]=22;		//number of data bytes in message
	
	process_reply_state=0;
	command_sent=CMD_READ_ACC_DATA;
	BNOnewdataready=0;
	usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
	usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
	
}

void read_BNO_euler_data(void)
{
	uint8_t command[4];
	
	for(BNOdata_pointer=0;BNOdata_pointer<6;BNOdata_pointer++)
	{
		BNOData[BNOdata_pointer]=0;		//clear all data
	}
	
	command[0]=0xAA;	//header
	command[1]=0x01;	//read command
	command[2]=GYROXADDR;	//EULER Address start
	command[3]=6;		//number of data bytes in message
	
	process_reply_state=0;
	command_sent=CMD_READ_EULER_DATA;
	BNOnewdataready=0;
	usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
	usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
	
}




void update_motors (void)
{
	motor1_out = +roll_factor - pitch_factor - yaw_factor + thrust_factor  ;
if (motor1_out < 30)
{
	motor1_out = 30;
}
	motor2_out = +roll_factor + pitch_factor + yaw_factor + thrust_factor  ;
if (motor2_out < 30)
{
	motor2_out = 30;
}
	motor3_out = -roll_factor - pitch_factor + yaw_factor + thrust_factor  ;
if (motor3_out < 30)
{
	motor3_out = 30;
}
	motor4_out = -roll_factor + pitch_factor - yaw_factor + thrust_factor  ;
if (motor4_out < 30)
{
	motor4_out = 30;
}
}

void accel_angles (void)
{
	accel_vector = atanf(sqrtf(((faccY*faccY)+(faccX*faccX))/faccZ));
	roll = (2*atanf(faccX/abs(faccY))*accel_vector)/PI;
	pitch = (2*atanf(faccY/abs(faccX))*accel_vector)/PI;
	
}

void get_euler_values(void)
{
	//eulerX=eulerX_MSB;
	//eulerX=eulerX << 8;
	//eulerX=eulerX | eulerX_LSB;
	//feulerX = (float) eulerX;
	//feulerX = feulerX*(360.0/5759.0);
	//
//
	//eulerY=eulerY_MSB;
	//eulerY=eulerY << 8;
	//eulerY=eulerY | eulerY_LSB;
	//feulerY = (float) eulerY;
	//
	//feulerY = feulerY*(360.0/5759.0);
	//
	//
	//eulerZ=eulerZ_MSB;
	//eulerZ=eulerZ << 8;
	//eulerZ=eulerZ | eulerZ_LSB;
	//feulerZ = (float) eulerZ;
	//
	//feulerZ = feulerZ*(360.0/5759.0);
	
	accX=acc_X_MSB;
	accX=accX << 8;
	accX= accX | acc_X_LSB;
	faccX = (float) accX;
	
	accY=acc_Y_MSB;
	accY=accY << 8;
	accY= accY | acc_Y_LSB;
	faccY = (float) accY;
	
	accZ=acc_Z_MSB;
	accZ=accZ << 8;
	accZ= accZ | acc_Z_LSB;
	faccZ = (float) accZ;
	
	gyrZ=gyr_X_MSB;
	gyrZ=gyrZ << 8;
	gyrZ= gyrZ | gyr_X_LSB;
	if (gyrZ < 20000 && gyrZ > -20000)
	{
		fgyrZ_last = fgyrZ;
	fgyrZ = (float) gyrZ;
	delta_gyrZ = fgyrZ_last-fgyrZ;
	fgyrZ_filtered = fgyrZ + filter_factor*(fgyrZ_filtered-fgyrZ);
	feulerZ_filtered = delta_gyrZ + euler_filter*(feulerZ_filtered-delta_gyrZ);
	}
	
	gyrY=gyr_Y_MSB;
	gyrY=gyrY << 8;
	gyrY= gyrY | gyr_Y_LSB;
	if (gyrY < 20000 && gyrY > -20000)
	{
		fgyrY_last = fgyrY;
	fgyrY = (float) gyrY;
	delta_gyrY = fgyrY_last - fgyrY;
	fgyrY_filtered = fgyrY + filter_factor*(fgyrY_filtered-fgyrY);
	feulerY_filtered = delta_gyrY + euler_filter*(feulerY_filtered-delta_gyrY);
	}
	
	gyrX=gyr_Z_MSB;
	gyrX=gyrX << 8;
	gyrX= gyrX | gyr_Z_LSB;
	fgyrX_last = fgyrX;
	fgyrX = (float) gyrX;
	delta_gyrX = fgyrX - fgyrX_last;
	
}

void read_calib_values(void)
{
	uint8_t command[4];
	
	for(BNOdata_pointer=0;BNOdata_pointer<22;BNOdata_pointer++)
	{
		BNOData[BNOdata_pointer]=0;		//clear all data
	}
	
	command[0]=0xAA;	//header
	command[1]=0x01;	//read command
	command[2]=ACCOFFSET;	//ACCOFFSET Address start
	command[3]=22;		//number of data bytes in message
	
	process_reply_state=0;
	command_sent=CMD_READ_CALIB_DATA;
	BNOnewdataready=0;
	usart_write_buffer_wait(&cdc_instance_BNO005, command, sizeof(command));
	usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
	
	
	
}

void configure_eeprom(void)
{
    /* Setup EEPROM emulator service */
    enum status_code error_code = eeprom_emulator_init();
    if (error_code == STATUS_ERR_NO_MEMORY) {
        while (true) {
            /* No EEPROM section has been set in the device's fuses */
        }
    }
    else if (error_code != STATUS_OK) {
        /* Erase the emulated EEPROM memory (assume it is unformatted or
         * irrecoverably corrupt) */
        eeprom_emulator_erase_memory();
        eeprom_emulator_init();
    }
}
#if (SAMD || SAMR21)
void SYSCTRL_Handler(void)
{
    if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
        SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33DET;
        eeprom_emulator_commit_page_buffer();
    }
}
#endif
static void configure_bod(void)
{
#if (SAMD || SAMR21)
    struct bod_config config_bod33;
    bod_get_config_defaults(&config_bod33);
    config_bod33.action = BOD_ACTION_INTERRUPT;
    /* BOD33 threshold level is about 3.2V */
    config_bod33.level = 48;
    bod_set_config(BOD_BOD33, &config_bod33);
    bod_enable(BOD_BOD33);
    SYSCTRL->INTENSET.reg = SYSCTRL_INTENCLR_BOD33DET;
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
#endif
}

uint8_t read_keypress (uint8_t key)
{
if(newkeydata==key) return 0x01;
else return 0x00;

}
uint8_t waitforkeypressed(uint8_t key)
{
	
	newkeydata=0;
	while(newkeydata==0){}
	
	if(newkeydata==key) return 0x01;
	else return 0x00;
	
}



///////////START OF MAIN ENTRY//////////////////////////////////////////

int main (void)
{
	uint16_t ii;
	struct port_config pin_conf;
	char key;

	
	// system initialization - includes, clock and board initialization.
	system_init();
	system_interrupt_enable_global();
	
	delay_init();
	
	port_get_config_defaults(&pin_conf);
	
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;		//blue LED
	port_pin_set_config(PIN_PA17, &pin_conf);
	port_pin_set_output_level(PIN_PA17, true);
	
	port_get_config_defaults(&pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;		//input switch 1
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA14, &pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;		//input switch 2
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA15, &pin_conf);


	//configure_eeprom();
	//configure_bod();

	port_pin_set_output_level(PIN_PA17,false);
	
	//configure_usart();
	//configure_usart_callbacks();
	//usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);
	
	configure_usart_cdc();
	
	configure_usart_callbacks();
	usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);
	printf("  Hello again !!! \r\n");
	configure_usart_BNO005();
	configure_BNO005_usart_callbacks();
	
	configure_usart_MATEK();
	configure_usart_callbacks_MATEK();
	//usart_read_job(&cdc_instance_MATEK,(uint8_t *)MATEK_rx_buffer);

	configure_tcc_zero();	//configure PWM generation for motors
	configure_tcc_one();	//configure thrust control input 1ms to 2ms signal period capture

	configu_eic();			//configure the external interrupt pin for the 1ms 2ms signal
	configure_evsys();		//map the external interrupt to the timer
	
	null_motor(0);			//ensure motors see a do nothing PWM signal
	null_motor(1);
	null_motor(2);
	null_motor(3);
	
	//printf("  Hello Anybody !!! \r\n");
	//key=' ';
	//while(1)
	//{
	//scanf("%c", (char *)&key);
	//if(key=='x') 	printf("  Hello again !!! \r\n");
	//else 	printf("  I'm off !!! \r\n");}


	usart_write_buffer_wait(&cdc_instance, hellostring, sizeof(hellostring));
	usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);
	delay_ms(2000);
	
	//uint8_t motorcalib[]="Beginning the Motor calibration, Press z to send high then x for low then c to complete\r\n";
	//usart_write_buffer_wait(&cdc_instance, motorcalib, sizeof(motorcalib));
	//usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);
	//
	//
	//waitforkeypressed('z');
	//
	//drive_motor(0,7990);
	//drive_motor(1,7990);
	//drive_motor(2,7990);
	//drive_motor(3,7990);
	//
	//waitforkeypressed('x');
	//
	//drive_motor(0,0);
	//drive_motor(1,0);
	//drive_motor(2,0);
	//drive_motor(3,0);
	//
	//waitforkeypressed('c');
	
	newkeydata=0;


	printf("use previous calabration\r\n");
	usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);
	if(waitforkeypressed('y')==true)
	
	{
		
		set_to_CONFIG_mode();
		delay_ms(1000);
		port_pin_set_output_level(PIN_PA17,true);
		eeprom_emulator_read_page(0, page_data);

		writetoBNObuffer[0]=0xAA;	//header
		writetoBNObuffer[1]=0x00;	//write command
		writetoBNObuffer[2]=ACCOFFSET;	//start of the calib data registers
		writetoBNObuffer[3]=22;		//number of data bytes in message
		
		for(ii=0;ii<22;ii++)
		{
			writetoBNObuffer[ii+4]=page_data[ii];
			printdata("E",page_data[ii]);
		}
		usart_write_buffer_wait(&cdc_instance_BNO005, writetoBNObuffer, sizeof(22+4));
		usart_read_job(&cdc_instance_BNO005,(uint8_t *)BNO005_rx_buffer);
		delay_ms(1000);
		uint8_t cdw[]="Calibration data written into 9DOF sensor\r\n";
		usart_write_buffer_wait(&cdc_instance, cdw, sizeof(cdw));
		//
		set_to_NDOF_mode();
		//
		delay_ms(1000);
		
	}
	else
	{
		port_pin_set_output_level(PIN_PA17,false);
		usart_write_buffer_wait(&cdc_instance, startingcalibrationstring, sizeof(startingcalibrationstring));
		usart_write_buffer_wait(&cdc_instance, calibregisterstring, sizeof(calibregisterstring));
		delay_ms(2000);
		//
		set_to_NDOF_mode();
		//
		delay_ms(100);
		//
		while(BNCalibStatus!=0xff)
		{
			read_calib_status();
			
			printbinary(BNCalibStatus);
			
			delay_cycles_ms(40);
			
		}
		port_pin_set_output_level(PIN_PA17,true);
		
		//
		set_to_CONFIG_mode();
		
		delay_ms(20);
		
		read_calib_values();  // Calibration values will be now stored in BNOData array 0 to 21
		delay_ms(40);
		for(ii=0;ii<22;ii++)
		{
			page_data[ii]=BNOData[ii];
			printdata("CALIB",page_data[ii]);
		}
		eeprom_emulator_write_page(0, page_data);
		eeprom_emulator_commit_page_buffer();		//write calibration data to EEPROM

		for(ii=0;ii<22;ii++)
		{
			page_data[ii]=0;
		}

		eeprom_emulator_read_page(0, page_data);

		for(ii=0;ii<22;ii++)
		{
			page_data[ii]=BNOData[ii];
			printdata("EPROM",page_data[ii]);
		}
		set_to_NDOF_mode();
		//
		delay_ms(100);

		
	}

	printf("ARM?\r\n");
	usart_read_job(&cdc_instance,(uint8_t *)rx_buffer);
	if(waitforkeypressed('y')!=true)
	{
		printf("not arming\r\n");
		while(1)
		{
			delay_ms(100);
		}
		
	}
	//read_all_BNO_data();
	//
	//delay_ms(2000);
	//
	//printdata("GYROXLSB",BNOData[0]);
	//printdata("GYROXMSB",BNOData[1]);
	//printdata("GYROYLSB",BNOData[2]);
	//printdata("GYROYMSB",BNOData[3]);
	//printdata("GYROZLSB",BNOData[4]);
	//printdata("GYROZMSB",BNOData[5]);
	//printdata("EULEXLSB",BNOData[6]);
	//printdata("EULEXMSB",BNOData[7]);
	//printdata("EULEYLSB",BNOData[8]);
	//printdata("EULEYMSB",BNOData[9]);
	//printdata("EULEZLSB",BNOData[10]);
	//printdata("EULEZMSB",BNOData[11]);
	//printdata("QUATWLSB",BNOData[12]);
	//printdata("QUATWMSB",BNOData[13]);
	//printdata("QUATXLSB",BNOData[14]);
	//printdata("QUATXMSB",BNOData[15]);
	//printdata("QUATYLSB",BNOData[16]);
	//printdata("QUATYMSB",BNOData[17]);
	//printdata("QUATZLSB",BNOData[18]);
	//printdata("QUATZMSB",BNOData[19]);
	//read_acc_gyr_data
	//
	//
	//
	newkeydata=0;
	
	
	while(1)
	{
		
		
		
		//eulerstatusflag = 2;
	//	delay_ms(8);
		//read_acc_gyr_data();
		
		//();
		///get_euler_values();
		
		//delay_ms(8);
		read_BNO_euler_data();
		get_euler_values();
		
		get_flysky_input();
		debug_counter++;
		//debug_counter2++;
		
		rate_profile();
		get_control_inputs();
		
		//accel_angles();
		
		//	harmonic_cancelation();
		//	get_control_inputs();
		safety_status();
		calculate_pid_output();
	
		
		update_motors();
		drive_motor(0,motor1_out);
		drive_motor(1,motor2_out);
		drive_motor(2,motor3_out);
		drive_motor(3,motor4_out);
	//	printf("%f , %f , %f, \n\r", fgyrY,fgyrZ,fgyrX);
		//printf("debug = %i\n\r" ,debug_counter);
		//printf(" %i , %i ,%i ,%i ,  \n\r",Channel1,Channel2,Channel3,Channel4);
		
	}
	while(1)
	{
		
		
		
		//eulerstatusflag = 2;
		//delay_ms(8);
		read_BNO_euler_data();
		//	delay_ms(8);
		//debug_counter++;
		get_euler_values();
		get_control_inputs();
		safety_status();
		calculate_pid_output();
		update_motors();
		
		drive_motor(0,motor1_out);
		drive_motor(1,motor2_out);
		drive_motor(2,motor3_out);
		drive_motor(3,motor4_out);
		

	}
}



//	gyro_drift=gyro_drift+fgyrY;




//	//printf("yaw = %f   pitch = %f   roll = %f   ryaw = %f   rpitch = %f   rroll = %f debug = %i gyrodrift = %f \n\r", feulerX,feulerY,feulerZ,fgyrX,fgyrY,fgyrZ,debug_counter,gyro_drift);
//printf("debug = %i\n\r",debug_counter);

//	if (feulerY > 0)
//	{
//		feulerY = 0;
//	}





//printbinary(eulerX_MSB);
//printdata(" Yaw: ",eulerX);
//
//printbinary(eulerY_LSB);
//printbinary(eulerY_MSB);
//printdata(" Roll: ",eulerX);
//
//printbinary(eulerZ_LSB);
//printbinary(eulerZ_MSB);
//printdata(" Pitch: ",eulerX);









//newkeydata=0;

//
//Roll_PID.Prev_Error=0;
//Roll_PID.Integral_Error=0;
//Pitch_PID.Prev_Error=0;
//Pitch_PID.Integral_Error=0;
//Yaw_PID.Prev_Error=0;
//Yaw_PID.Integral_Error=0;
//
//T[0]=0.0;
//T[1]=0.0;
//T[2]=0.0;
//T[3]=0.0;
//
//Remote_Throttle=0.1;
//
//Roll_PID.KP=0.1;
//Roll_PID.KD=0.1;
//Roll_PID.KI=0.1;
//
//Pitch_PID.KP=0.1;
//Pitch_PID.KD=0.1;
//Pitch_PID.KI=0.1;
//
//Yaw_PID.KP=0.1;
//Yaw_PID.KD=0.1;
//Yaw_PID.KI=0.1;
//
//Modifide_eulerX=0.0;
//read_BNO_euler_data();
//delay_ms(10);
//get_euler_values();
//
//last_euler=eulerX;
//
//
//while(1)
//{
//read_BNO_euler_data();
////delay_ms(10);
//get_euler_values();
////printf ("eauler_yaw= %f \n\r", feulerX);
////printsigneddata("Pitch:",-eulerY);
////printsigneddata("Roll:",eulerZ);
//
//
//Modifide_eulerX=Modifide_eulerX + (eulerX - last_euler);
//
//Run_Control_Loop();
//
//uint16_t IntegerT0=T[0];
//uint16_t IntegerT1=T[1];
//uint16_t IntegerT2=T[2];
//uint16_t IntegerT3=T[3];
//
//drive_motor(1,IntegerT0);
//drive_motor(0,IntegerT1);
//drive_motor(3,IntegerT2);
//drive_motor(2,IntegerT3);
//}

//if(newkeydata!=0)
//{
//switch(newkeydata)
//{
//case 'z':		//reduce thrust
//Remote_Throttle--;
//break;
//
//case 'x':		//increase thrust
//Remote_Throttle++;
//break;
//
//case 'a':		//increase proportional term
//Roll_PID.KP+=0.1;
//
//Pitch_PID.KP+=0.1;
//
//Yaw_PID.KP+=0.1;
//break;
//case 's':		//decrease proportional term
//if(Roll_PID.KP > 0.0) Roll_PID.KP-=0.1;
//
//if(Pitch_PID.KP > 0.0) Pitch_PID.KP-=0.1;
//
//if(Yaw_PID.KP > 0.0) Yaw_PID.KP-=0.1;
//
//break;
//
//
//case '1':
//
////drive motor 0 null all others
//
//break;
//
//case '2':
//
////drive motor 1 null all others
//
//break;
//
//case '3':
//
////drive motor 2 null all others
//
//break;
//
//case '4':
//
////drive motor 3 null all others
//
//break;
//
//
//
//
//}
//newkeydata=0;
//}
//
//
//}
//
//
//
//
//}

