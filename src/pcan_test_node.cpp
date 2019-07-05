#include <pcan_test/can_config.h>

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

#include <net/if.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/epoll.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <ros/ros.h>



extern int optind, opterr, optopt;

static int	s = -1;
static int	running = 1;

int epid;
struct epoll_event event;
struct epoll_event events[6];

enum {
	VERSION_OPTION = CHAR_MAX + 1,
	FILTER_OPTION,
};

static void print_usage(char *prg)
{
        fprintf(stderr, "Usage: %s [<can-interface>] [Options]\n"
		"Options:\n"
		" -f, --family=FAMILY\t"	"protocol family (default PF_CAN = %d)\n"
		" -t, --type=TYPE\t"		"socket type, see man 2 socket (default SOCK_RAW = %d)\n"
		" -p, --protocol=PROTO\t"	"CAN protocol (default CAN_RAW = %d)\n"
		"     --filter=id:mask[:id:mask]...\n"
		"\t\t\t"			"apply filter\n"
		" -h, --help\t\t"		"this help\n"
		" -o <filename>\t\t"		"output into filename\n"
		" -d\t\t\t"			"daemonize\n"
		"     --version\t\t"		"print version information and exit\n",
		prg, PF_CAN, SOCK_RAW, CAN_RAW);
}

static void sigterm(int signo)
{
	running = 0;
}

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

int EpollInit(int cfd){
	epid = epoll_create(6);

	event.events = EPOLLET | EPOLLIN;
	event.data.fd = cfd;
	if(epoll_ctl(epid, EPOLL_CTL_ADD, cfd, &event) != 0){
		printf("set epoll error!\n");
		return 0;
	}
	printf("set epoll ok!\n");

	return 1;


}



#define BUF_SIZ	(255)

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcan_test_node");

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

	signal(SIGPIPE, SIG_IGN);

	struct option		long_options[] = {
		{ "help", no_argument, 0, 'h' },
		{ "family", required_argument, 0, 'f' },
		{ "protocol", required_argument, 0, 'p' },
		{ "type", required_argument, 0, 't' },
		{ "filter", required_argument, 0, FILTER_OPTION },
		{ "version", no_argument, 0, VERSION_OPTION},
		{ 0, 0, 0, 0},
	};

	while ((opt = getopt_long(argc, argv, "f:t:p:o:d", long_options, NULL)) != -1) {
		switch (opt) {
		case 'd':
			optdaemon++;
			break;

		case 'h':
			print_usage(basename(argv[0]));
			exit(0);

		case 'f':
			family = strtoul(optarg, NULL, 0);
			break;

		case 't':
			type = strtoul(optarg, NULL, 0);
			break;

		case 'p':
			proto = strtoul(optarg, NULL, 0);
			break;

		case 'o':
			optout = optarg;
			break;

		case FILTER_OPTION:
			ptr = optarg;
			while(1) {
				id = strtoul(ptr, NULL, 0);
				ptr = strchr(ptr, ':');
				if(!ptr) {
					fprintf(stderr, "filter must be applied in the form id:mask[:id:mask]...\n");
					exit(1);
				}
				ptr++;
				mask = strtoul(ptr, NULL, 0);
				ptr = strchr(ptr, ':');
				add_filter(id,mask);
				if(!ptr)
					break;
				ptr++;
			}
			break;

		case VERSION_OPTION:
			printf("candump %s\n",VERSION);
			exit(0);

		default:
			fprintf(stderr, "Unknown option %c\n", opt);
			break;
		}
	}

	if (optind != argc)
		interface = argv[optind];

	printf("interface = %s, family = %d, type = %d, proto = %d\n",
	       interface, family, type, proto);

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

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}
	EpollInit(s);
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

	while (running) {
		int witeNum = epoll_wait(epid,events,1,50);
		printf("witeNum = %d\n",witeNum);
		if(witeNum == 0)
			continue;
		else{
			for(int i = 0;i < witeNum; i++){
				if((events[i].events & EPOLLERR)
						|| (events[i].events & EPOLLHUP)
						|| (!(events[i].events & EPOLLIN))){
					printf("no data\n");
					break;
				}
				else if(events[i].events & EPOLLIN){
					if ((nbytes = read(s, &frame, sizeof(struct can_frame))) < 0) {
								perror("read");
								return 1;
							} else {
								if (frame.can_id & CAN_EFF_FLAG)
									n = snprintf(buf, BUF_SIZ, "<0x%08x> ", frame.can_id & CAN_EFF_MASK);
								else
									n = snprintf(buf, BUF_SIZ, "<0x%03x> ", frame.can_id & CAN_SFF_MASK);

								n += snprintf(buf + n, BUF_SIZ - n, "[%d] ", frame.can_dlc);
								for (i = 0; i < frame.can_dlc; i++) {
									n += snprintf(buf + n, BUF_SIZ - n, "%02x ", frame.data[i]);
								}
								if (frame.can_id & CAN_RTR_FLAG)
									n += snprintf(buf + n, BUF_SIZ - n, "remote request");

								fprintf(out, "%s\n", buf);

								do {
									err = fflush(out);
									if (err == -1 && errno == EPIPE) {
										err = -EPIPE;
										fclose(out);
										out = fopen(optout, "a");
										if (!out)
											exit (EXIT_FAILURE);
									}
								} while (err == -EPIPE);

								n = 0;
							}

				}
			}

		}

	}

	exit (EXIT_SUCCESS);
}
