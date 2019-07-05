#include <pcan_test/can_config.h>
#include <pcan_test/joint_data_type.h>

#include <errno.h>
#include <getopt.h>
#include <libgen.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <stdint.h>
#include <fcntl.h>

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fstream>


#include <linux/can.h>
#include <linux/can/raw.h>

//thread
#include <pthread.h>
//ros
#include <ros/ros.h>
#define TO_SLAVE_HAND_PORT 12625
#define FROM_SLAVE_HAND_PORT 17362

#define TO_MATLAB_PORT 9090

extern int optind, opterr, optopt;
// socketcan_descriptor
static int	s = -1;
static int	running = 1;
//socketudp_descriptor
int socket_udp;
struct sockaddr_in addr_udp;
//configuration
unsigned int cfg_UploadID_list[16];
unsigned int cfg_DownloadID_list[16];
unsigned int cfg_Data_offset[16];
// global real time joint data;
JOINT_DAT_TYPE joint_data;

enum {
	VERSION_OPTION = CHAR_MAX + 1,
	FILTER_OPTION,
};

static struct can_filter *filter = NULL;
static int filter_count = 0;

int add_filter(u_int32_t id, u_int32_t mask)
{
	filter = ( can_filter *)realloc((void*)filter, sizeof(struct can_filter) * (filter_count + 1));
	if(!filter)
		return -1;

	filter[filter_count].can_id = id;
	filter[filter_count].can_mask = mask;
	filter_count++;
	printf("id: 0x%08x mask: 0x%08x\n",id,mask);
	return 0;
}

static void sigterm(int signo)
{
	running = 0;
}

int set_nonblocking(int fd){
	int flags;

	if((flags = fcntl(fd,F_GETFL,0)) == -1)
		flags = 0;
	return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

#define BUF_SIZ	(255)
void* thread_udp(void* arg){
	unsigned int runcount = 0;
	int socketudp_frommaster = *(int*)arg;
	int socketudp_toslave;
	int socketudp_tomatlab;
	JOINT_DAT_TYPE recv_frame;
	int i = 0;

	struct sockaddr_in addr_toslave;
	bzero(&addr_toslave,sizeof(addr_toslave));
	addr_toslave.sin_family = AF_INET;
	addr_toslave.sin_addr.s_addr = inet_addr("127.0.0.1");
	addr_toslave.sin_port = htons(TO_SLAVE_HAND_PORT);

	socketudp_toslave = socket(AF_INET,SOCK_DGRAM,0);

	//to matlab
	struct sockaddr_in addr_tomatlab;
	bzero(&addr_tomatlab,sizeof(addr_tomatlab));
	addr_tomatlab.sin_family = AF_INET;
	addr_tomatlab.sin_addr.s_addr = inet_addr("192.168.1.187");
	addr_tomatlab.sin_port = htons(TO_MATLAB_PORT);

	socketudp_tomatlab = socket(AF_INET,SOCK_DGRAM,0);
	//end of to matlab
	while(running){
		//usleep(10);
		//printf("thread2 awake\r\n");
		socklen_t addr_udp_len = sizeof(addr_udp);
		int nbytes = recvfrom(socketudp_frommaster,&recv_frame, sizeof(JOINT_DAT_TYPE), 0, (struct sockaddr*)&addr_udp,(socklen_t*)&addr_udp_len);
		//printf("%d: received a pack!\r\n",runcount++);
		sendto(socketudp_toslave,&joint_data, sizeof(JOINT_DAT_TYPE), 0, (struct sockaddr*)&addr_toslave,addr_udp_len);
		int joint_data_matlab[7];
		for(i = 0; i < 7; i++){
			joint_data_matlab[i] = joint_data.joint_pos_abs[i];
		}
		sendto(socketudp_tomatlab,joint_data_matlab, sizeof(joint_data_matlab), 0, (struct sockaddr*)&addr_tomatlab,addr_udp_len);

		if(runcount++ % 100 == 0)
			for(i = 0; i < 16; i++)
				joint_data.joint_online[i] = false;
	}

}
void* thread_can(void* arg)
{
	int socketcan_fd = *(int*)arg;
	int s_can;
	struct can_frame frame;
	FILE *out = stdout;
	char *optout = NULL;
	char buf[BUF_SIZ];
	int n = 0, err;
	int nbytes, i;
	unsigned int runcount = 0;

	while(running){
		if ((nbytes = read(socketcan_fd, &frame, sizeof(struct can_frame))) < 0)
		{
			printf("read error\r\n");
		} else {

			unsigned int canid = frame.can_id;
			int dat,mag;
			int keyvalue;
			// use highest byte for key value, and low 3 bytes for encoder value
			dat = (unsigned char)frame.data[1] *65536 + (unsigned char)frame.data[2] * 256 + (unsigned char)frame.data[3];
			keyvalue = (unsigned char)frame.data[0];
			for(i = 0; i < 16; i++){
			        if(canid == cfg_UploadID_list[i]){
			            joint_data.joint_online[i] = true;
						joint_data.joint_pos_raw[i] = dat;
						joint_data.joint_pos_valid[i] = true;
						joint_data.keyvalue[i] = keyvalue;

						if(joint_data.joint_pos_raw[i] >= cfg_Data_offset[i])
							joint_data.joint_pos_abs[i] = joint_data.joint_pos_raw[i] - cfg_Data_offset[i];
						else
							joint_data.joint_pos_abs[i] = joint_data.joint_pos_raw[i] + 4096 - cfg_Data_offset[i];

						//datstr.setNum(t);
						//LETable[i]->setText(datstr);
						std::cout 	<< i << ":"
									<< joint_data.joint_pos_abs[i] << ":"
									<< joint_data.joint_online[i] << ":"
									<< joint_data.joint_pos_valid[i] << std::endl;

			            break;
			        }



			    }

		}
	}
	return NULL;
}

void get_config(void){
	int i,j;

	std::ifstream cfg("/home/zl/paramlist.cfg");
	if(!cfg.is_open()){
		perror("bad configuration!");
		exit(0);
	}

	cfg.seekg(0);

	for(i = 0; i < 16 && !cfg.eof(); i++){

			char line[256];
			std::string linestr;
			cfg.getline(line,256);
			linestr = line;
			char* p;
			std::string list[4];
			p = strtok(line,(const char*)",");
			j = 0;
			while(p) {
				list[j++] = p;
				p = strtok(NULL, (const char*)",");

			}

	        cfg_UploadID_list[i] = std::atoi(list[1].c_str());
	        cfg_Data_offset[i] = std::atoi(list[2].c_str());
	        cfg_DownloadID_list[i] = std::atoi(list[3].c_str());
	        /*
	        std::cout 	<< list[0] << ":"
						<< list[1] << ":"
						<< list[2] << ":"
						<< list[3] << "\r\n";
	        					*/
	        std::cout 	<< list[0] << ":"
						<< cfg_UploadID_list[i] << ":"
						<< cfg_Data_offset[i] << ":"
						<< cfg_DownloadID_list[i] << std::endl;

		}

	    if(i != 16){
	    	perror("bad configuration!");
	    	exit(0);
	    }

	    cfg.close();




}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcan_exo_node");

	struct can_frame frame;
	struct ifreq ifr;
	struct sockaddr_can addr;
	FILE *out = stdout;
	char *interface = "can0";
	char *optout = NULL;
	char *ptr;
	char buf[BUF_SIZ];
	int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
	int n = 0, err;
	int nbytes, i;
	int opt, optdaemon = 0;
	uint32_t id, mask;
	get_config();

	signal(SIGPIPE, SIG_IGN);



	printf("interface = %s, family = %d, type = %d, proto = %d\n",
	       interface, family, type, proto);

//create the socket
	if ((s = socket(family, type, proto)) < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family = family;
	strncpy(ifr.ifr_name, interface, sizeof(ifr.ifr_name));
	if (ioctl(s, SIOCGIFINDEX, &ifr)) {
		perror("ioctl");
		return 1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;
//bind socket
	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	if (filter) {
		if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filter,
			       filter_count * sizeof(struct can_filter)) != 0) {
			perror("setsockopt");
			exit(1);
		}
	}

	if (optdaemon)
		daemon(1, 0);
	else {
		signal(SIGTERM, sigterm);
		signal(SIGHUP, sigterm);
	}

	if (optout) {
		out = fopen(optout, "a");
		if (!out) {
			perror("fopen");
			exit (EXIT_FAILURE);
		}
	}
	/////////////////////////////////////////////////////////
	//set_nonblocking(s);

	//udp socket init

	bzero(&addr_udp,sizeof(addr_udp));
	addr_udp.sin_family = AF_INET;
	addr_udp.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("192.168.1.100");
	addr_udp.sin_port = htons(FROM_SLAVE_HAND_PORT);

	socket_udp = socket(AF_INET,SOCK_DGRAM,0);
	bind(socket_udp,(struct sockaddr*)&addr_udp,sizeof(addr_udp));

	//create child thread
	pthread_t tid_can,tid_udp;
	int exitCode_can,exitCode_udp;
	pthread_create(&tid_can,NULL,thread_can,(void*)&s);
	//pthread_create(&tid_can,NULL,thread_can,NULL);
	pthread_create(&tid_udp,NULL,thread_udp,(void*)&socket_udp);

	pthread_join(tid_can,(void**)&exitCode_can);
	pthread_join(tid_udp,(void**)&exitCode_udp);

	while (running) {

	}

	exit (EXIT_SUCCESS);
}
