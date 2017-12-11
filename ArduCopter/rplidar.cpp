#include "Copter.h"
#include "rplidar.h"

_rplidar rplidar;

#define hal_uart hal.uartC
#define hal_uart_send(x,l) hal_uart->write(x,l)
#define hal_uart_avail() hal_uart->available()
#define hal_uart_read() hal_uart->read()
#define delay_ms(x) hal.scheduler->delay(x)
#define hal_uart_printf hal.uartE->printf

#define D_BUFFER_MM  300
#define R_MAX   4500
void rplidar_get_input(float &target_roll,float &target_pitch,int16_t ROL_MIN,int16_t PIT_MIN,float scale)
{
    if(!rplidar.read_begin||(!ROL_MIN&&!PIT_MIN)||(scale==0)) return;
    int16_t rol=0,pit=0,tmp=0;
    int16_t rpl_roll_out,rpl_pitch_out,out_buffer,tar_buffer;
    uint16_t rpl_dis[4];
    rplidar_getdist(rpl_dis);
    ROL_MIN*=10;
    PIT_MIN*=10;

    tmp=PIT_MIN-rpl_dis[0];
    if(tmp>0)    pit+=tmp;
    tmp=PIT_MIN-rpl_dis[1];
    if(tmp>0)    pit-=tmp;

    tmp=ROL_MIN-rpl_dis[2];
    if(tmp>0)    rol+=tmp;
    tmp=ROL_MIN-rpl_dis[3];
    if(tmp>0)    rol-=tmp;

    rpl_roll_out  = (float)rol/(float)ROL_MIN*(float)R_MAX*(float)scale;
    rpl_pitch_out = (float)pit/(float)PIT_MIN*(float)R_MAX*(float)scale;

//
    out_buffer=rpl_roll_out;    tar_buffer=target_roll;
    if(out_buffer!=0){
        if((out_buffer>0&&tar_buffer<=0)||(out_buffer<0&&tar_buffer>=0))  target_roll=out_buffer;
    }else{
        if((((rpl_dis[2]>ROL_MIN)&&(rpl_dis[2]<(ROL_MIN+D_BUFFER_MM)))&&tar_buffer<0)||(((rpl_dis[3]>ROL_MIN)&&(rpl_dis[3]<(ROL_MIN+D_BUFFER_MM)))&&tar_buffer>0)){
            target_roll=0;
        }
    }

    out_buffer=rpl_pitch_out;    tar_buffer=target_pitch;
    if(out_buffer!=0){
        if((out_buffer>0&&tar_buffer<=0)||(out_buffer<0&&tar_buffer>=0))  target_pitch=out_buffer;
    }else{
        if((((rpl_dis[2]>PIT_MIN)&&(rpl_dis[2]<(PIT_MIN+D_BUFFER_MM)))&&tar_buffer<0)||(((rpl_dis[3]>PIT_MIN)&&(rpl_dis[3]<(PIT_MIN+D_BUFFER_MM)))&&tar_buffer>0)){
            target_pitch=0;
        }
    }

    if(target_roll>R_MAX)    target_roll=R_MAX;
    else if(target_roll<-R_MAX)    target_roll=-R_MAX;
    if(target_pitch>R_MAX)    target_pitch=R_MAX;
    else if(target_pitch<-R_MAX)    target_pitch=-R_MAX;
/*
    static uint8_t cnt;
    if((++cnt)>=240){cnt=0;
//        copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING,"#r:%d %d %0.2f rt:%0.0f\n",rpl_roll_out,rol,rs,target_roll);}
        copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING,"#r:%d rt:%0.0f p:%d pt:%0.0f\n",rpl_roll_out,target_roll,rpl_pitch_out,target_pitch);}
*/
}

void rplidar_init(void)
{
    hal_uart->begin(115200,504,512);
    hal_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    delay_ms(1000);
	rplidar_read(RP_STOP);
    if(rplidar_read(GET_HEALTH)) hal.uartE->printf("#read health ok\n");
    else hal.uartE->printf("#cant read health\n");
//	if(rplidar.status==2){
//		rplidar_read(RP_RESET);
//	}
	rplidar_read(RP_STOP);
    if(rplidar_read(EXPRESS_SCAN)) hal.uartE->printf("#start express scan\n");
    else hal.uartE->printf("#cant express scan\n");
}

uint8_t rplidar_read(uint8_t type)
{
	uint8_t data1[]={0xa5,0x52};//GET_HEALTH
	uint8_t data2[]={0xa5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22};//EXPRESS_SCAN
	uint8_t data3[]={0xa5,0x40};//RESET
	uint8_t data4[]={0xa5,0x25};//STOP
	uint8_t data5[]={0xa5,0x20};//SCAN
    uint8_t data[200];
    uint8_t ret=0,i=0;
	switch(type)
	{
		case GET_HEALTH://GET_HEALTH
            hal_uart_send(data1,sizeof(data1));delay_ms(10);
            hal_uart_printf("#avail: %d \n",hal_uart_avail());
            while(hal_uart_avail()){
                data[i++]=hal_uart_read();
                if(i>=200)break;
            }
            ret = rev_GET_HEALTH(data,i);hal_uart_printf("#avail2: %d %u\n",hal_uart_avail(),i);
            if(ret){
                rplidar.status=data[7];
                rplidar.errorcode=((data[9]<<8)|data[8]);
            }
		break;
        case EXPRESS_SCAN://EXPRESS_SCAN
            hal_uart_send(data2,sizeof(data2));delay_ms(10);
            hal_uart_printf("#avail: %d \n",hal_uart_avail());
            while(hal_uart_avail()){
                data[i++]=hal_uart_read();
                if(i>=200)break;
            }
            ret = rev_EXPRESS_SCAN(data,i);hal_uart_printf("#avail2: %d %u\n",hal_uart_avail(),i);
            rplidar.read_begin=ret;
        break;
        case SCAN://SCAN
            hal_uart_send(data5,sizeof(data5));
            while(hal_uart_avail()){
                data[i++]=hal_uart_read();
                if(i>=20)break;
            }
            ret = rev_SCAN(data,i);
            rplidar.read_begin=ret;
		break;
        case RP_RESET://RESET
            hal_uart_send(data3,sizeof(data3));
            delay_ms(3000);
		break;
		case RP_STOP://STOP
            hal_uart_send(data4,sizeof(data4));
            while(hal_uart_avail()){hal_uart_read();}
            delay_ms(3000);
            while(hal_uart_avail()){hal_uart_read();}
		break;
        default:ret=0;break;
	}
	return ret;
}

uint8_t rev_GET_HEALTH(uint8_t* data,uint8_t len)
{
	uint8_t i,cnt=0;
	for(i=0;i<len;i++){
		switch(cnt){
			case 0:
				if(data[i]==0xa5) cnt=1;
				else cnt=0;
				break;
			case 1:
				if(data[i]==0x5a) cnt=2;
				else cnt=0;
				break;
			case 2:
				if(data[i]==0x03) cnt=3;
				else cnt=0;
				break;
			case 3:
				if(data[i]==0x00) cnt=4;
				else cnt=0;
				break;
			case 4:
				if(data[i]==0x00) cnt=5;
				else cnt=0;
				break;
			case 5:
				if(data[i]==0x00) cnt=6;
				else cnt=0;
				break;
			case 6:
				if(data[i]==0x06){
					return 1;
				}
				cnt=0;
				break;
			default: cnt=0;
		}
	}
	return 0;
}
uint8_t rev_EXPRESS_SCAN(uint8_t* data,uint8_t len)
{
	uint8_t i,cnt=0;
	for(i=0;i<len;i++){
		switch(cnt){
			case 0:
				if(data[i]==0xa5) cnt=1;
				else cnt=0;
				break;
			case 1:
				if(data[i]==0x5a) cnt=2;
				else cnt=0;
				break;
			case 2:
				if(data[i]==0x54) cnt=3;
				else cnt=0;
				break;
			case 3:
				if(data[i]==0x00) cnt=4;
				else cnt=0;
				break;
			case 4:
				if(data[i]==0x00) cnt=5;
				else cnt=0;
				break;
			case 5:
				if(data[i]==0x40) cnt=6;
				else cnt=0;
				break;
			case 6:
				if(data[i]==0x82){
					return 1;
				}
				cnt=0;
				break;
			default: cnt=0;
		}
	}
	return 0;
}
uint8_t rev_SCAN(uint8_t* data,uint8_t len)
{
	uint8_t i,cnt=0;
	for(i=0;i<len;i++){
		switch(cnt){
			case 0:
				if(data[i]==0xa5) cnt=1;
				else cnt=0;
				break;
			case 1:
				if(data[i]==0x5a) cnt=2;
				else cnt=0;
				break;
			case 2:
				if(data[i]==0x05) cnt=3;
				else cnt=0;
				break;
			case 3:
				if(data[i]==0x00) cnt=4;
				else cnt=0;
				break;
			case 4:
				if(data[i]==0x00) cnt=5;
				else cnt=0;
				break;
			case 5:
				if(data[i]==0x40) cnt=6;
				else cnt=0;
				break;
			case 6:
				if(data[i]==0x81){
					return 1;
				}
				cnt=0;
				break;
			default: cnt=0;
		}
	}
	return 0;
}
uint8_t rplidar_checksum(uint8_t *data)
{
    if(((data[0]&0xf0)!=0xa0)&&((data[1]&0xf0)!=0x50)) {return 0;}
    uint8_t recvChecksum = ((data[0] & 0xF) | (data[1]<<4));
    uint8_t i,checksum=0;
    for(i=2;i<84;++i){
        checksum^=data[i];
    }
    if(checksum!=recvChecksum)
    {return 0;}
    return 1;
}

uint16_t test_times[10];
static uint8_t databuff[504];
static uint16_t datalen;
void rplidar_rev(void)
{
/*    static uint8_t cnt;
    if((++cnt)>20){
        cnt=0;
        hal_uart_printf("#0:%d #1:%d #2:%d #3:%d #4:%d #5:%d #6:%d ava:%d\n",test_times[0],test_times[1],test_times[2],test_times[3],test_times[4],test_times[5],test_times[6],hal_uart_avail());
    }
*/

    static uint8_t data_timeout_cnt=0;
    if((++data_timeout_cnt)>=10){
        data_timeout_cnt=0;
        for(uint16_t a=0;a<360;a++){
            if(!rplidar.realdata_quality[a])   rplidar.realdata[a]=7995;
            rplidar.realdata_quality[a]=0;
        }
    }
    if(!rplidar.read_begin) return;
    uint16_t pos,i,timeout=0;
    while(hal_uart_avail()){
        if((++timeout)==500) return;
        databuff[0]=hal_uart_read();
        if((databuff[0]  &0xf0)!=0xa0) continue;
        databuff[1]=hal_uart_read();
        if((databuff[1]&0xf0)!=0x50) continue;
        datalen=hal_uart_avail()/84;
        if(datalen>6) datalen=6;
        datalen=datalen*84;
        if(datalen<84) continue;
        for(i=2;i<datalen;++i){
            databuff[i]=hal_uart_read();
        }
        break;
    }
    for(pos=0;pos<datalen;++pos){
        if((datalen-pos)<84) {test_times[4]++;break;}
        if((databuff[pos]  &0xf0)!=0xa0) {test_times[3]++;continue;}
        if((databuff[pos+1]&0xf0)!=0x50) {test_times[2]++;continue;}
        for(i=0;i<84;++i){
            rplidar.recv_data[i]=databuff[pos++];
        }pos--;
        if(rplidar_handle(rplidar.recv_data)){
            test_times[0]++;
        }else test_times[1]++;
    }
}

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
uint8_t rplidar_handle(uint8_t *buff_data)
{
    if(!rplidar_checksum(buff_data)) return 0;
    memcpy(&rplidar.rplidar_measure,buff_data,84);
	
	static _rplidar_capsule_measurement _cached_previous_capsuledata;
	int diffAngle_q8;
	int currentStartAngle_q8 = ((rplidar.rplidar_measure.start_angle_sync_q6 & 0x7FFF)<< 2);
	int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);
	
	diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
	if (prevStartAngle_q8 >  currentStartAngle_q8) {
			diffAngle_q8 += (360<<8);
	}
	int angleInc_q16 = (diffAngle_q8 << 3);
	int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
	
	for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos){
		
		int dist_q2[2];
		int angle_q6[2];
		int syncBit[2];

		dist_q2[0] = (((_cached_previous_capsuledata.cabins[pos].distance_angle1[1]<<8)|_cached_previous_capsuledata.cabins[pos].distance_angle1[0]) &0XFFFC);
		dist_q2[1] = (((_cached_previous_capsuledata.cabins[pos].distance_angle2[1]<<8)|_cached_previous_capsuledata.cabins[pos].distance_angle2[0]) & 0xFFFC);

		int angle_offset1_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles & 0xF) | ((((_cached_previous_capsuledata.cabins[pos].distance_angle1[1]<<8)|_cached_previous_capsuledata.cabins[pos].distance_angle1[0]) & 0x3)<<4));
		int angle_offset2_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles >> 4) | ((((_cached_previous_capsuledata.cabins[pos].distance_angle2[1]<<8)|_cached_previous_capsuledata.cabins[pos].distance_angle2[0]) & 0x3)<<4));

		angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3<<13))>>10);
		syncBit[0] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
		currentAngle_raw_q16 += angleInc_q16;

		angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3<<13))>>10);
		syncBit[1] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
		currentAngle_raw_q16 += angleInc_q16;

		for (int cpos = 0; cpos < 2; ++cpos) {
				if (angle_q6[cpos] < 0) angle_q6[cpos] += (360<<6);
				if (angle_q6[cpos] >= (360<<6)) angle_q6[cpos] -= (360<<6);

				_rplidar_measurement node;

				node.sync_quality = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
				if (dist_q2[cpos]) node.sync_quality |= (0x2F << 2);
				node.sync_quality>>=2;
			
				node.angle = (1 | (angle_q6[cpos]<<1));
				node.angle = angle_q6[cpos]/64;
			
				node.distance = dist_q2[cpos]/4;
						  
				if(node.angle<360){
					if(node.distance&&node.sync_quality){                        
                        rplidar.realdata[node.angle] = node.distance;
                        rplidar.realdata_quality[node.angle] = 1;
                    }
                }
		 }
   }	
    _cached_previous_capsuledata=rplidar.rplidar_measure;
    return 1;
}

uint16_t mid_data(uint16_t angle,uint8_t n)
{
    uint8_t i,j;
    uint16_t data[n],k;
    if(angle<n) angle=360-n/2;
    for(i=0;i<n;i++){
        if(angle>=360)  angle=0;
        data[i]=rplidar.realdata[angle++];
    }
    for(i=0;i<n;i++){
        for(j=0;j<(n-i);j++){
            if(data[j]>data[j+1]){
                k=data[j];
                data[j]=data[j+1];
                data[j+1]=k;
            }
        }
    }
    for(i=0;i<n;i++){
        if(data[i]>120)return data[i];
    }
    return 7887;
}
#define NUM 20
void get_data_ave(uint16_t* data)
{
    static uint16_t data_ave[4][NUM];
    static uint8_t index;
    uint32_t data_sum;

    for(uint8_t k=0;k<4;k++){
        data_ave[k][index]=data[k];
        data_sum=0;
        for(uint8_t i=0;i<NUM;i++){
            data_sum+=data_ave[k][i];
        }
        data[k]=data_sum/NUM;
    }
    if((++index)>=NUM) index=0;
}
void rplidar_getdist(uint16_t *data)
{
    data[0]=mid_data(0,9);
    data[1]=mid_data(180,9);
    data[2]=mid_data(90,9);
    data[3]=mid_data(270,9);
    get_data_ave(data);
    return;
    data[0]=rplidar.realdata[0];
    data[1]=rplidar.realdata[180];
    data[2]=rplidar.realdata[90];
    data[3]=rplidar.realdata[270];
    return;
}

double KalmanFilter(unsigned int DataCnt,double *Data,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
{
        unsigned int i;
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;
        double x_last = *Data;
        double x_mid = x_last;
        double x_now=0;
        double p_last = InitialPrediction;
        double p_mid ;
        double p_now;
        double kg;

        for(i=0;i<DataCnt;i++)
        {
                x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
                p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
                kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
//                z_measure=z_real+frand()*0.03;//测量值
                x_now=x_mid+kg*(*(Data+i)-x_mid);//估计出的最优值
                p_now=(1-kg)*p_mid;//最优值对应的covariance

                p_last = p_now; //更新covariance值
                x_last = x_now; //更新系统状态值
        }
        return x_now;

}













/*
    if(angle<90){
        left=dist*sin(angle);
        front=dist*cos(angle);
    }else if(angle<180){ angle-=90;
        back=dist*sin(angle);
        left=dist*cos(angle);
    }else if(angle<270){  angle-=180;
        right=dist*sin(angle);
        back=dist*cos(angle);
    }else if(angle<360){ angle-=270;
        front=dist*sin(angle);
        right=dist*cos(angle);
    }
*/
