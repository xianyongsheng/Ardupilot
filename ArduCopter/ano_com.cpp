#include "ano_com.h" 

static uint8_t ano_send_buff[50];

#define ano_data_send(s,l)	if(hal.uartC->txspace()>l)hal.uartC->write(s,l)

#define ANO_SDATUS	0.0f,1.0f,2.0f,0,0,0

void  ano_exchange_run(void)
{
    //ano_send_status(ANO_SDATUS);
    uint8_t data2[]={0xa5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22};
    ano_data_send(data2,sizeof(data2));
}

void ano_receive_parse(uint8_t *data_buf,uint8_t num)
{
	uint8_t i,sum = 0;
	for(i=0;i<(num-1);i++) sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))	return;		
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	return;

	switch(*(data_buf+2)){		
		case 0x01:
			if(*(data_buf+4)==0X01){	
				
			}
			else if(*(data_buf+4)==0X02){	
				
			}
			else if(*(data_buf+4)==0X03){

			}else if(*(data_buf+4)==0X04){

			}else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26)){

			}
			else if(*(data_buf+4)==0X20){

			}
		break;
		case 0x02:
		if(*(data_buf+4)==0X01){

		}else if(*(data_buf+4)==0X02){
			
		}else if(*(data_buf+4)==0XA0){

		}else if(*(data_buf+4)==0XA1){

		}
	break;
	case 0x03:

	break;
	case 0x10://PID1
		ano_send_ack(*(data_buf+2),sum);
	break;
	case 0x11://PID2
	ano_send_ack(*(data_buf+2),sum);
	break;
	case 0x12://PID3
		ano_send_ack(*(data_buf+2),sum);
	break;
	case 0x13://PID4
		ano_send_ack(*(data_buf+2),sum);
	break;
	case 0x14://PID5
		ano_send_ack(*(data_buf+2),sum);
	break;
	case 0x15://PID6
		ano_send_ack(*(data_buf+2),sum);
	break;
	case 0x16:

	break;
	case 0x17:

	break;
	}
}


void ano_receive_decode(uint8_t data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA){
		state=1;
		RxBuffer[0]=data;
	}else if(state==1&&data==0xAF){
		state=2;
		RxBuffer[1]=data;
	}else if(state==2&&data<0XF1){
		state=3;
		RxBuffer[2]=data;
	}else if(state==3&&data<50){
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}else if(state==4&&_data_len>0){
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0) state = 5;
	}else if(state==5){
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ano_receive_parse(RxBuffer,_data_cnt+5);
	}else state = 0;
}

void send_data(uint8_t *dataToSend , uint8_t length)
{
    ano_data_send(dataToSend,length);
}
void ano_send_ack(uint8_t head, uint16_t check)
{
	ano_send_buff[0]=0xAA;
	ano_send_buff[1]=0xAA;
	ano_send_buff[2]=0xEF;
	ano_send_buff[3]=2;
	ano_send_buff[4]=head;
	ano_send_buff[5]=check;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += ano_send_buff[i];
	ano_send_buff[6]=sum;

	send_data(ano_send_buff,8);
}

void ano_send_vel(float x_s,float y_s,float z_s)
{
	uint8_t _cnt=0;
	int16_t _temp;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x0B;
	ano_send_buff[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);
	
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	ano_send_buff[_cnt++]=sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_gps(uint8_t state,uint8_t sat_num,int32_t lon,int32_t lat,float back_home_angle)
{
	uint8_t _cnt=0;
	int16_t _temp;
	int32_t _temp2;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x04;
	ano_send_buff[_cnt++]=0;
	
	ano_send_buff[_cnt++]=state;
	ano_send_buff[_cnt++]=sat_num;
	
	_temp2 = lon;
	ano_send_buff[_cnt++]=BYTE3(_temp2);
	ano_send_buff[_cnt++]=BYTE2(_temp2);	
	ano_send_buff[_cnt++]=BYTE1(_temp2);
	ano_send_buff[_cnt++]=BYTE0(_temp2);
	
	_temp2 = lat;
	ano_send_buff[_cnt++]=BYTE3(_temp2);
	ano_send_buff[_cnt++]=BYTE2(_temp2);	
	ano_send_buff[_cnt++]=BYTE1(_temp2);
	ano_send_buff[_cnt++]=BYTE0(_temp2);
	
	
	_temp = (int16_t)(100 *back_home_angle);
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);
	
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	ano_send_buff[_cnt++]=sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	int16_t _temp;
	int32_t _temp2 = alt;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x01;
	ano_send_buff[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);
	
	ano_send_buff[_cnt++]=BYTE3(_temp2);
	ano_send_buff[_cnt++]=BYTE2(_temp2);
	ano_send_buff[_cnt++]=BYTE1(_temp2);
	ano_send_buff[_cnt++]=BYTE0(_temp2);
	
	ano_send_buff[_cnt++] = fly_model;
	
	ano_send_buff[_cnt++] = armed;
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	ano_send_buff[_cnt++]=sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_imu(int16_t* senser)
{
	uint8_t _cnt=0;
	int16_t _temp;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x02;
	ano_send_buff[_cnt++]=0;
	
	for(uint8_t i=0;i<9;i++){
		ano_send_buff[_cnt++]=BYTE1(senser[i]);
		ano_send_buff[_cnt++]=BYTE0(senser[i]);
	}
/////////////////////////////////////////
	_temp = 0;	
	ano_send_buff[_cnt++]=BYTE1(_temp);
	ano_send_buff[_cnt++]=BYTE0(_temp);	
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	ano_send_buff[_cnt++] = sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_alt(int32_t bar_alt,uint16_t csb_alt)
{
	uint8_t _cnt=0;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x07;
	ano_send_buff[_cnt++]=0;
	
	ano_send_buff[_cnt++]=BYTE3(bar_alt);
	ano_send_buff[_cnt++]=BYTE2(bar_alt);
	ano_send_buff[_cnt++]=BYTE1(bar_alt);
	ano_send_buff[_cnt++]=BYTE0(bar_alt);

	ano_send_buff[_cnt++]=BYTE1(csb_alt);
	ano_send_buff[_cnt++]=BYTE0(csb_alt);
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	ano_send_buff[_cnt++] = sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_rc(uint16_t* rc)
{
	uint8_t _cnt=0;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x03;
	ano_send_buff[_cnt++]=0;
	
	for(uint8_t i=0;i<10;i++){
		ano_send_buff[_cnt++]=BYTE1(rc[i]);
		ano_send_buff[_cnt++]=BYTE0(rc[i]);
	}

	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	
	ano_send_buff[_cnt++]=sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_batt(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x05;
	ano_send_buff[_cnt++]=0;
	
	ano_send_buff[_cnt++]=BYTE1(votage);
	ano_send_buff[_cnt++]=BYTE0(votage);
	ano_send_buff[_cnt++]=BYTE1(current);
	ano_send_buff[_cnt++]=BYTE0(current);
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	
	ano_send_buff[_cnt++]=sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_motor(uint16_t motor[8])
{
	uint8_t _cnt=0;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x06;
	ano_send_buff[_cnt++]=0;
	
	for(uint8_t k=0;k<8;k++){
		ano_send_buff[_cnt++]=BYTE1(motor[k]);
		ano_send_buff[_cnt++]=BYTE0(motor[k]);
	}
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	
	ano_send_buff[_cnt++]=sum;
	
	send_data(ano_send_buff, _cnt);
}

void ano_send_pid(uint8_t group,float* pid)
{
	uint8_t _cnt=0;
	int16_t _temp;
	
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x10+group-1;
	ano_send_buff[_cnt++]=0;
	
	for(uint8_t k=0;k<9;k++){
		_temp = pid[k] * 1000;
		ano_send_buff[_cnt++]=BYTE1(_temp);
		ano_send_buff[_cnt++]=BYTE0(_temp);
	}
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	
	ano_send_buff[_cnt++]=sum;

	send_data(ano_send_buff, _cnt);
}

void ano_send_user(uint8_t id, float* ano_user_data, uint8_t len)
{
	uint8_t _cnt=0;
	int16_t _temp;
	ano_send_buff[_cnt++]=0xAA; 
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=id;
	ano_send_buff[_cnt++]=0;
		
	for(uint8_t i=0;i<len;i++){
		_temp = (int16_t)ano_user_data[i];					
		ano_send_buff[_cnt++]=BYTE1(_temp);
		ano_send_buff[_cnt++]=BYTE0(_temp);
	}
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	
	ano_send_buff[_cnt++]=sum;

	send_data(ano_send_buff, _cnt);
}

void ano_send_version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0xAA;
	ano_send_buff[_cnt++]=0x00;
	ano_send_buff[_cnt++]=0;
	
	ano_send_buff[_cnt++]=hardware_type;
	ano_send_buff[_cnt++]=BYTE1(hardware_ver);
	ano_send_buff[_cnt++]=BYTE0(hardware_ver);
	ano_send_buff[_cnt++]=BYTE1(software_ver);
	ano_send_buff[_cnt++]=BYTE0(software_ver);
	ano_send_buff[_cnt++]=BYTE1(protocol_ver);
	ano_send_buff[_cnt++]=BYTE0(protocol_ver);
	ano_send_buff[_cnt++]=BYTE1(bootloader_ver);
	ano_send_buff[_cnt++]=BYTE0(bootloader_ver);
	
	ano_send_buff[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += ano_send_buff[i];
	ano_send_buff[_cnt++]=sum;
	
	send_data(ano_send_buff, _cnt);
}


/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
