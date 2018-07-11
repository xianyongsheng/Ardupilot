#include "Copter.h"

void ano_exchange_run(void);

void ano_receive_decode(uint8_t data);
void ano_receive_parse(uint8_t *data_buf,uint8_t num);

void send_data(uint8_t *dataToSend , uint8_t length);
void ano_send_ack(uint8_t head, uint16_t check);

void ano_send_user(uint8_t id, float* ano_user_data, uint8_t len);//user_data*len
void ano_send_pid(uint8_t group,float* pid);//pid[3]*3
void ano_send_motor(uint16_t motor[8]);//motor*8
void ano_send_batt(uint16_t votage, uint16_t current);
void ano_send_rc(uint16_t* rc);//thr,yaw,rol,pit,aux1*6
void ano_send_alt(int32_t bar_alt,uint16_t csb_alt);
void ano_send_imu(int16_t* senser);//acc,gyro,mag
void ano_send_status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
void ano_send_gps(uint8_t state,uint8_t sat_num,int32_t lon,int32_t lat,float back_home_angle);
void ano_send_vel(float x_s,float y_s,float z_s);
void ano_send_version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

