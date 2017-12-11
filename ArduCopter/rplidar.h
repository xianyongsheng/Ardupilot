

typedef struct _rplidar_response_measurement_node_t {
    uint8_t    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    uint16_t   angle; // check_bit:1;angle_q6:15;
    uint16_t   distance;
}_rplidar_measurement;
typedef struct _rplidar_response_cabin_nodes_t {
    uint8_t   distance_angle1[2]; // see [distance_sync flags]
    uint8_t   distance_angle2[2]; // see [distance_sync flags]
    uint8_t   offset_angles;  
}_rplidar_cabin;   
typedef struct _rplidar_response_capsule_measurement_nodes_t {
    uint8_t                             s_checksum_1; // see [s_checksum_1]
    uint8_t                             s_checksum_2; // see [s_checksum_1]
    uint16_t                            start_angle_sync_q6;
    _rplidar_cabin  cabins[16];
}_rplidar_capsule_measurement;
typedef struct  {
    uint16_t   distance1; // see [distance_sync flags]
    uint16_t   distance2; // see [distance_sync flags]
    uint16_t    angles1; 
uint16_t    angles2; 	
} _distance;   

typedef struct  {
  uint8_t read_begin;	
	uint8_t status;
	uint16_t errorcode;
	_rplidar_capsule_measurement rplidar_measure;
	_distance data[16];
        uint8_t recv_data[84];
	uint16_t realdata[360];	
        uint8_t  realdata_quality[360];
}_rplidar;


#define RP_RESET 	0
#define RP_STOP		1
#define GET_HEALTH 	2
#define EXPRESS_SCAN 	3
#define SCAN            4

void rplidar_init(void);
void rplidar_run(void);
void rplidar_rev(void);
uint8_t rplidar_handle(uint8_t *buff_data);
uint8_t rev_SCAN(uint8_t* data,uint8_t len);
uint8_t rev_EXPRESS_SCAN(uint8_t* data,uint8_t len);
uint8_t rev_GET_HEALTH(uint8_t* data,uint8_t len);
uint8_t rplidar_read(uint8_t type);
uint8_t rplidar_checksum(uint8_t *data);
void rplidar_getdist(uint16_t *data);
void rplidar_get_input(float &target_roll, float &target_pitch, int16_t ROL_MIN, int16_t PIT_MIN, float scale);
uint16_t mid_data(uint16_t angle,uint8_t n);
