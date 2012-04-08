#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <libgen.h>
#include <errno.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "sensor_cmd.h"

#define CMD_NAME		"lemon_squashd"
#define REMOCON_PORT_STR	"26851"
#define SENSOR_PORT_STR		"26852"
#define BAUDRATE		B115200

#define VERBOSE

#ifdef VERBOSE
#define debug_syslog	syslog
#else	/* VERBOSE */
#define debug_syslog(...)	do {} while (0)
#endif	/* VERBOSE */

static struct {
	const char *dev_name;
} app;

struct server_fds {
	fd_set readfds;
	int fd_ser;
	int fd_sock_rm;	/* REMOCON server socket */
	int fd_conn_rm;	/* REMOCON client connection */
	int fd_sock_sn;	/* SENSOR server socket */
	int fd_conn_sn;	/* SENSOR client connection */
};

volatile int had_signal = 0;

static void handler(int signo)
{
	syslog(LOG_NOTICE, "signal %d received.\n", signo);
	had_signal = 1;
	if (signo == SIGKILL)
		exit(0);
}

static void setup_signal(void)
{
	struct sigaction sigact;

	sigact.sa_handler = handler;

	/* SIGTERM, SIGKILL */
	__sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = SA_NOMASK | SA_ONESHOT;
	sigact.sa_restorer = NULL;
	sigaction(SIGTERM, &sigact, NULL);

	__sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = SA_NOMASK | SA_ONESHOT;
	sigact.sa_restorer = NULL;
	sigaction(SIGKILL, &sigact, NULL);
}

static int daemonize(void)
{
	pid_t pid;

	if ((pid = fork()) < 0) {
		fprintf(stderr, "fork failed\n");
		syslog(LOG_ERR, "fork failed\n");
		return pid;
	}
	if (pid != 0)
		exit(0);	/* parent exits here */

	/* child */
	setsid();	/* session leader */
	chdir("/");
	umask(0);

	/* close stdios */
	close(0);
	close(1);
	close(2);

	return 0;
}

static int serial_open(struct server_fds *fds,
		       const char *devname, struct termios *tio_old)
{
	struct termios tio_new;
	int fd;

	fd = open(devname, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		fprintf(stderr, "device open failed: %s (%s)\n",
			devname, strerror(errno));
		return -1;
	}

	tcgetattr(fd, tio_old);

	tio_new.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	tio_new.c_iflag = 0;
	tio_new.c_oflag = 0;
	tio_new.c_lflag = 0;
	tio_new.c_cc[VMIN] = 1;
	tio_new.c_cc[VTIME] = 0;

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &tio_new);

	fds->fd_ser = fd;
	return 0;
}

static int serial_close(struct server_fds *fds, const struct termios *tio_old)
{
	tcsetattr(fds->fd_ser, TCSANOW, tio_old);
	return close(fds->fd_ser);
}

static int server_open(struct server_fds *fds)
{
	int fd;
	struct addrinfo ai_hint, *aip;
	int r;

	memset(&ai_hint, 0, sizeof(struct addrinfo));
	ai_hint.ai_family = AF_INET;
	ai_hint.ai_socktype = SOCK_STREAM;
	ai_hint.ai_flags = AI_PASSIVE;
	ai_hint.ai_protocol = 0;
	ai_hint.ai_canonname = NULL;
	ai_hint.ai_addr = NULL;
	ai_hint.ai_next = NULL;

	/*
	 * REMOCON port
	 */
	if ((r = getaddrinfo(NULL, REMOCON_PORT_STR, &ai_hint, &aip)) < 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(r));
		return -1;
	}

	if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		fprintf(stderr, "socket() failed\n");
		return -1;
	}

	if (bind(fd, aip->ai_addr, aip->ai_addrlen) < 0) {
		fprintf(stderr, "bind() failed\n");
		close(fd);
		freeaddrinfo(aip);
		return -1;
	}

	freeaddrinfo(aip);

	if (listen(fd, 1) < 0) {
		fprintf(stderr, "listen() failed\n");
		close(fd);
		return -1;
	}

	fds->fd_sock_rm = fd;

	/*
	 * SENSOR port
	 */
	if ((r = getaddrinfo(NULL, SENSOR_PORT_STR, &ai_hint, &aip)) < 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(r));
		return -1;
	}

	if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		fprintf(stderr, "socket() failed\n");
		return -1;
	}

	if (bind(fd, aip->ai_addr, aip->ai_addrlen) < 0) {
		fprintf(stderr, "bind() failed\n");
		close(fd);
		freeaddrinfo(aip);
		return -1;
	}

	freeaddrinfo(aip);

	if (listen(fd, 1) < 0) {
		fprintf(stderr, "listen() failed\n");
		close(fd);
		return -1;
	}

	fds->fd_sock_sn = fd;

	return 0;
}

static int server_close(struct server_fds *fds)
{
	int r1, r2;

	r1 = close(fds->fd_sock_rm);
	r2 = close(fds->fd_sock_sn);
	return (r1 < 0) ? r1 : r2;
}

static int read_server_sock_rm(struct server_fds *fds)
{
	struct sockaddr_in addr;
	size_t len = sizeof(struct sockaddr_in);

	debug_syslog(LOG_INFO, "connection from remocon client\n");
	fds->fd_conn_rm =
		accept(fds->fd_sock_rm, (struct sockaddr *)&addr, &len);
	if (fds->fd_conn_rm < 0) {
		syslog(LOG_ERR, "accept() failed\n");
		return -1;
	}
	debug_syslog(LOG_INFO, "fd_conn_rm = %d\n", fds->fd_conn_rm);
	FD_CLR(fds->fd_sock_rm, &fds->readfds);
	FD_SET(fds->fd_conn_rm, &fds->readfds);
	return 0;
}

static int read_server_sock_sn(struct server_fds *fds)
{
	struct sockaddr_in addr;
	size_t len = sizeof(struct sockaddr_in);

	debug_syslog(LOG_INFO, "connection from sensor client\n");
	fds->fd_conn_sn =
		accept(fds->fd_sock_sn, (struct sockaddr *)&addr, &len);
	if (fds->fd_conn_sn < 0) {
		syslog(LOG_ERR, "accept() failed\n");
		return -1;
	}
	debug_syslog(LOG_INFO, "fd_conn_sn = %d\n", fds->fd_conn_sn);
	FD_CLR(fds->fd_sock_sn, &fds->readfds);
	FD_SET(fds->fd_conn_sn, &fds->readfds);
	return 0;
}

/* forwarding remocon client -> serial */
static int read_client_conn_rm(struct server_fds *fds)
{
	char buf[256];
	ssize_t r_len = recv(fds->fd_conn_rm, buf, sizeof(buf), 0);

	debug_syslog(LOG_INFO, "data at fd_conn_rm\n");
	if (r_len <= 0) {
		if (r_len < 0)	/* maybe client died? */
			syslog(LOG_ERR, "recv() failed\n");
		else
			debug_syslog(LOG_INFO, "connection closed\n");
		close(fds->fd_conn_rm);
		FD_CLR(fds->fd_conn_rm, &fds->readfds);
		fds->fd_conn_rm = -1;
		FD_SET(fds->fd_sock_rm, &fds->readfds);
	} else {
		/* FIXME: command check? */
		if (write(fds->fd_ser, buf, r_len) < r_len) {
			syslog(LOG_ERR, "write() failed\n");
			return -1;
		}
	}

	return 0;
}

/* forwarding sensor client -> serial */
static int read_client_conn_sn(struct server_fds *fds)
{
	char cmd_r, cmd_s;
	ssize_t r_len = recv(fds->fd_conn_sn, &cmd_r, 1, 0);

	debug_syslog(LOG_INFO, "data at fd_conn_sn\n");
	if (r_len <= 0) {
		if (r_len < 0)	/* maybe client died? */
			syslog(LOG_ERR, "recv() failed\n");
		else
			debug_syslog(LOG_INFO, "connection closed\n");
		close(fds->fd_conn_sn);
		FD_CLR(fds->fd_conn_sn, &fds->readfds);
		fds->fd_conn_sn = -1;
		FD_SET(fds->fd_sock_sn, &fds->readfds);
	} else {
		switch (cmd_r) {
		case SENSOR_CMD_START:
			cmd_s = 'Q';
			break;
		case SENSOR_CMD_ACTIVE:
			cmd_s = 'A';
			break;
		case SENSOR_CMD_INACTIVE:
			cmd_s = 'B';
			break;
		default:
			syslog(LOG_ERR, "unknown command %d\n", cmd_r);
			return 0;
		}
		if (write(fds->fd_ser, &cmd_s, 1) < 1) {
			syslog(LOG_ERR, "write() failed\n");
			return -1;
		}
	}

	return 0;
}

/* forwarding serial -> client */
static int read_serial(struct server_fds *fds)
{
	char buf[256];
	ssize_t r_len = read(fds->fd_ser, buf, sizeof(buf));

	debug_syslog(LOG_INFO, "data at fd_ser\n");
	if (r_len < 0) {
		syslog(LOG_ERR, "read() failed\n");
		return -1;
	}

	/* FIXME: more strict data parsing */
	if ((r_len == 1) && (buf[0] == 'P')) {
		if (fds->fd_conn_sn < 0) {
			syslog(LOG_ERR,
			       "data at fd_ser, but no connection with"
			       " sensor client. discarding.\n");
			syslog(LOG_ERR,
			       "data[0] = 0x%02x\n", buf[0]);
		} else {
			char cmd_s = SENSOR_CMD_DETECTED;
			if (send(fds->fd_conn_sn, &cmd_s, 1, 0) < 1) {
				syslog(LOG_ERR, "send() failed\n");
				return -1;
			}
		}
	} else {
		if (fds->fd_conn_rm < 0) {
			syslog(LOG_ERR,
			       "data at fd_ser, but no connection with"
			       " remocon client. discarding.\n");
			syslog(LOG_ERR,
			       "data[0] = 0x%02x\n", buf[0]);
		} else {
			if (send(fds->fd_conn_rm, buf, r_len, 0) < r_len) {
				syslog(LOG_ERR, "send() failed\n");
				return -1;
			}
		}
	}

	return 0;
}

static int read_all_fds(fd_set *testfds, struct server_fds *fds)
{
	if (FD_ISSET(fds->fd_sock_rm, testfds)) {
		if (read_server_sock_rm(fds) < 0)
			return -1;
	}
	if (FD_ISSET(fds->fd_sock_sn, testfds)) {
		if (read_server_sock_sn(fds) < 0)
			return -1;
	}
	if ((fds->fd_conn_rm >= 0) && FD_ISSET(fds->fd_conn_rm, testfds)) {
		if (read_client_conn_rm(fds) < 0)
			return -1;
	}
	if ((fds->fd_conn_sn >= 0) && FD_ISSET(fds->fd_conn_sn, testfds)) {
		if (read_client_conn_sn(fds) < 0)
			return -1;
	}
	if (FD_ISSET(fds->fd_ser, testfds)) {
		if (read_serial(fds) < 0)
			return -1;
	}

	return 0;
}

static void server(struct server_fds *fds)
{
	syslog(LOG_INFO, "start\n");

	sleep(2);	/* wait for arduino serial setup */

	/* initialize readfds */
	FD_ZERO(&fds->readfds);
	FD_SET(fds->fd_ser, &fds->readfds);
	FD_SET(fds->fd_sock_rm, &fds->readfds);
	FD_SET(fds->fd_sock_sn, &fds->readfds);

	for (;;) {
		fd_set rfds = fds->readfds;
		int sel_result;

		sel_result = select(FD_SETSIZE, &rfds, NULL, NULL, NULL);
		if (had_signal)
			break;

		if (sel_result < 0) {
			syslog(LOG_ERR, "select() failed: %d\n", errno);
			break;
		} else if (sel_result == 0) {
			/* timeout: no way */
		} else {
			/* some of fds are set */
			if (read_all_fds(&rfds, fds) < 0)
				break;
		}
	}

	syslog(LOG_INFO, "exitting.\n");
}

static void usage(const char *cmd_path)
{
	char *cpy_path = strdup(cmd_path);

	fprintf(stderr,
"usage : %s\n"
"        -h                     :help\n"
"        -s <serial devilce>    :specify serial device (default=/dev/ttyACM0)\n",
		basename(cpy_path));
	free(cpy_path);
}

static int parse_arg(int argc, char **argv)
{
	int i;

	/* init */
	app.dev_name = "/dev/ttyACM0";

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-h")) {
			usage(argv[0]);
			exit(0);
		} else if (!strcmp(argv[i], "-s")) {
			if (++i == argc)
				goto err;
			app.dev_name = argv[i];
		} else
			goto err;
	}

	return 0;

err:
	usage(argv[0]);
	return -1;
}


int main(int argc, char **argv)
{
	struct server_fds fds = {
		.fd_ser = -1,
		.fd_sock_rm = -1,
		.fd_conn_rm = -1,
		.fd_sock_sn = -1,
		.fd_conn_sn = -1,
	};
	struct termios tio_old;

	if (parse_arg(argc, argv) < 0)
		return 1;

	if (serial_open(&fds, app.dev_name, &tio_old) < 0)
		return 1;

	if (server_open(&fds) < 0)
		return 1;

	daemonize();
	setup_signal();
	openlog(CMD_NAME, 0, LOG_DAEMON);
	server(&fds);
	closelog();

	server_close(&fds);
	serial_close(&fds, &tio_old);

	return 0;
}
