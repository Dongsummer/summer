
# summer
NOTE:
I am summer,it's my first time use git and feel good.'
Thanks



#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <uORB/topics/mcu_avoid.h>
#include <uORB/topics/summer.h>
//test
#include <uORB/topics/test.h>

///* 定义主题 */
//ORB_DEFINE(rw_uart_sonar, struct rw_uart_sonar_data_s);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int mcu_avoid_main(int argc, char *argv[]);
int mcu_avoid_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static



int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [param]\n\n");
    exit(1);
}

int mcu_avoid_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("mcu_avoid",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         mcu_avoid_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

int mcu_avoid_thread_main(int argc, char *argv[])
{

    if (argc < 2) {
        errx(1, "[YCM]need a serial port name as argument");
        usage("eg:");
    }

    const char *uart_name = argv[1];

    warnx("[YCM]opening port %s", uart_name);
//    char data = '0';
//    char buffer[12] = "";
//   static uint16_t distance = 0;//距离
//    static uint8_t rate = 0;//危险等级
//    static double angle = 0;//角度
//    static double orient = 0;//参考角度
//    static uint8_t checksum = 0;//校验和
//    static uint8_t checks = 0;//实际校验

    //char data[100] = "";
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init(uart_name);
    if(false == uart_read) uart_read = false;//return -1;
    if(false == set_uart_baudrate(uart_read,57600)){
        printf("[YCM]set_uart_baudrate is failed\n");
        //return -1;
    }
    printf("[YCM]uart init is successful\n");

    thread_running = true;


//###########################
   // struct mcu_avoid_s avoiddate;
   // memset(&avoiddate, 0 , sizeof(avoiddate));
    //orb_advert_t mcu_avoid_pub = orb_advertise(ORB_ID(mcu_avoid), &avoiddate);


    //test
    struct summer_s summerdata;
    memset(&summerdata, 0 , sizeof(summerdata));
    orb_advert_t summer_pub = orb_advertise(ORB_ID(summer), &summerdata);
    static int aaaa = 1;

    //test
        struct test_s data;
        memset(&data, 0 , sizeof(data));
        orb_advert_t test_pub = orb_advertise(ORB_ID(test), &data);

//#############################

    while(!thread_should_exit){
//        read(uart_read,&data,1);
//        //printf("this is %c\t%d\n%c\t%d\n",data1[0],data1[1]);
//        data = '0';
//
//        printf("this is first:\t%d\n",data);
//        if((data == 136)){
//        	data = '0';
//        	read(uart_read,&data,1);
//        	buffer[1] = 136;
//            if((data == 136)){
//            		//printf("it is ok in the first step.\n");
//            		buffer[2] = 136;
//					for(int k = 0;k<8;k++)
//					{
//						data = '0';
//						read(uart_read,&data,1);
////						printf("it is data[%d] :\t%d\n",k+3,data);
//						buffer[k+3] = data-1;
//
//					}
//
//					 	 	 distance = (uint16_t)buffer[3] + (uint16_t)buffer[4]*256;
//					        angle =(double)( (uint16_t)buffer[6] + (uint16_t)buffer[7]*256);
//					        angle = angle / 10;
//					        rate = buffer[5] ;
//					        orient = (double)((uint16_t)buffer[8]+(uint16_t)buffer[9]*256);
//					        orient = orient / 10;
//					        checksum = buffer[10];
//					        checks = 0;
//					        for(int k = 1;k<=9;k++)
//					        {
//					        	checks = checks + buffer[k];
//					        }
//					        if(checksum == checks)
//					        	{
////										printf("everything is OK!\n");
////										printf("this is distance: %d\t rate: %d\t angle: %.2f\t orint: %.2f\t checksum:%d\t che:%d\n",distance,rate,angle,orient,checksum,checks);
//
//
//					        //##################################
//					                            avoiddate.distance = distance;
//					                            avoiddate.angle =angle;
//					                            avoiddate.risk_i = rate;
//					                            avoiddate.ang_sugg =orient;
//					        	avoiddate.distance = 12;
//					            avoiddate.angle =23;
//                          avoiddate.risk_i = 1;
//	                       avoiddate.ang_sugg =45;
//
//					                            orb_publish(ORB_ID(mcu_avoid), mcu_avoid_pub, &avoiddate);
//										}
//
//            }
//        }
       // avoiddate.distance = 1;
//      avoiddate.angle =23+(double)aaaa*0.1;
       // avoiddate.angle =2;
       // avoiddate.risk_i = 3;
       // avoiddate.ang_sugg =4;
       // avoiddate.summer = 10;


        summerdata.summer = 6;
        summerdata.data = 7;


        data.data1 = 1;
        data.data2 = 2;
        data.data3 = 3;

        orb_publish(ORB_ID(summer), summer_pub, &summerdata);
        orb_publish(ORB_ID(test), test_pub, &data);
       // orb_publish(ORB_ID(mcu_avoid), mcu_avoid_pub, &avoiddate);
        usleep(100000);
        aaaa++;




//        	if(data == 'X'){
//        		char checkdata = 0;
//                    for(int i = 0;i <9;++i){
//                       data = '0';
//                       read(uart_read,&data,1);
//                        buffer[i] = data;
//
//                        printf("buffer[%d]:\t%d\t",i,buffer[i]);
//
//                         checkdata = checkdata + buffer[i];
//
//
//
//                    }
//                    //printf("\n check is :%d\n",checkdata);
//                    checkdata = checkdata + 'X'+'X';
//                    if(checkdata == buffer[9])
//                    {
//                    	printf("check is passed");
//                    }
//
//                    printf("\n check is :%d\n",checkdata);
//
//        	}
//        	else
//        	{
//        			printf("missing the data.\n");
//        	}

//        if(data == '1')
//        {
//        	printf("it is ok");
//        }
/*        if(data == 'R'){
            for(int i = 0;i <4;++i){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
                printf("this is buffer[%d]:\t%d\n",i,buffer[i]);
            }*/
//            strncpy(sonardata.datastr,buffer,4);
//            sonardata.data = atoi(sonardata.datastr);
            //printf("[YCM]sonar.data=%d\n",buffer);
//            orb_publish(ORB_ID(rw_uart_sonar), rw_uart_sonar_pub, &sonardata);
            //char* bala ="1234678990";
            //buffer[0] = 'c';
           // buffer[1] = 'q';
         //   buffer[2] = '4';
           // buffer[3] = 't';


            //strncpy(bala,buffer,4);
            //rdatatest.angle = atoi(bala);

            //printf("this is data : %d\n",(int)buffer[2]);
//#########################
//            			static int mmmm = 1;
//
//                        avoiddate.distance = (int)(2+mmmm*0.01);
//                        avoiddate.angle =(float)(4.3+mmmm*0.008);
//                        avoiddate.risk_i = 2;
//                        orb_publish(ORB_ID(mcu_avoid), mcu_avoid_pub, &avoiddate);
//                        mmmm++;
//############################

//            mavlink_distance_sensor_t msg;


//            msg.time_boot_ms = 16;
//            msg.type = 2;
//            msg.orientation = 16;
//            msg.id = 1;
//            msg.min_distance = 12.5; /* m to cm */
//            msg.max_distance = 27.43; /* m to cm */
//            msg.current_distance = 23.22; /* m to cm */
//            msg.covariance = 4;

//            mavlink_msg_distance_sensor_send_struct(1, &msg);



//        static int mmmm = 1;
//
//                    avoiddate.distance = (int)(2+mmmm*0.1);
//                    avoiddate.angle =(float)(4.3+mmmm*0.08);
//                    avoiddate.risk_i = 2;
//                    orb_publish(ORB_ID(mcu_avoid), mcu_avoid_pub, &avoiddate);
//                    mmmm++;
    }

    warnx("[YCM]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}
