/*
 gcc -pthread -o seq_demo seq_demo.c -lrt
 ./seq_demo
*/

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>
#include <time.h>        /* for clock_gettime */
#include <signal.h>      /* for signal handling */
#include <sys/poll.h>    /* for poll() */
#include <fcntl.h>       /* for open() */

#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

static int file_i2c = 0;
// Sensor calibration data
static int calT1,calT2,calT3;
static int calP1, calP2, calP3, calP4, calP5, calP6, calP7, calP8, calP9;
static int calH1, calH2, calH3, calH4, calH5, calH6;

int bme280Init(int iChannel, int iAddr);
int bme280ReadValues(int *T, int *P, int *H);

int bme280Init(int iChannel, int iAddr)
{
    int i, rc;
    unsigned char ucTemp[32];
    unsigned char ucCal[36];
    char filename[32];
    
	sprintf(filename, "/dev/i2c-%d", iChannel);
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		fprintf(stderr, "Failed to open the i2c bus\n");
		file_i2c = 0;
		return -1;
	}

	if (ioctl(file_i2c, I2C_SLAVE, iAddr) < 0)
	{
		fprintf(stderr, "Failed to acquire bus access or talk to slave\n");
		file_i2c = 0;
		return -1;
	}

	ucTemp[0] = 0xd0; // get ID
	rc = write(file_i2c, ucTemp, 1);
	i = read(file_i2c, ucTemp, 1);
	if (rc < 0 || i != 1 || ucTemp[0] != 0x60)
	{
		printf("Error, ID doesn't match 0x60; wrong device?\n");
		return -1;
	}
	// Read 24 bytes of calibration data
	ucTemp[0] = 0x88; // starting 4from register 0x88
	rc = write(file_i2c, ucTemp, 1);
	i = read(file_i2c, ucCal, 24);
	if (rc < 0 || i != 24)
	{
		printf("calibration data not read correctly\n");
		return -1;
	}
	ucTemp[0] = 0xa1; // get humidity calibration byte
	rc = write(file_i2c, ucTemp, 1);
	i = read(file_i2c, &ucCal[24], 1);
	ucTemp[0] = 0xe1; // get 7 more humidity calibration bytes
	rc = write(file_i2c, ucTemp, 1);
	i = read(file_i2c, &ucCal[25], 7);
	if (rc < 0 || i < 0) // something went wrong
	{}
	// Prepare temperature calibration data
	calT1 = ucCal[0] + (ucCal[1] << 8);
	calT2 = ucCal[2] + (ucCal[3] << 8);
	if (calT2 > 32767) calT2 -= 65536; // negative value
	calT3 = ucCal[4] + (ucCal[5] << 8);
	if (calT3 > 32767) calT3 -= 65536;
	
	// Prepare pressure calibration data
	calP1 = ucCal[6] + (ucCal[7] << 8);
	calP2 = ucCal[8] + (ucCal[9] << 8);
	if (calP2 > 32767) calP2 -= 65536; // signed short
	calP3 = ucCal[10] + (ucCal[11] << 8);
	if (calP3 > 32767) calP3 -= 65536;
	calP4 = ucCal[12] + (ucCal[13] << 8);
	if (calP4 > 32767) calP4 -= 65536;
	calP5 = ucCal[14] + (ucCal[15] << 8);
	if (calP5 > 32767) calP5 -= 65536;
	calP6 = ucCal[16] + (ucCal[17] << 8);
	if (calP6 > 32767) calP6 -= 65536;
	calP7 = ucCal[18] + (ucCal[19] << 8);
	if (calP7 > 32767) calP7 -= 65536;
	calP8 = ucCal[20] + (ucCal[21] << 8);
	if (calP8 > 32767) calP8 -= 65536;
	calP9 = ucCal[22] + (ucCal[23] << 8);
	if (calP9 > 32767) calP9 -= 65536;

	// Prepare humidity calibration data
	calH1 = ucCal[24];
	calH2 = ucCal[25] + (ucCal[26] << 8);
	if (calH2 > 32767) calH2 -= 65536;
	calH3 = ucCal[27];
	calH4 = (ucCal[28] << 4) + (ucCal[29] & 0xf);
	if (calH4 > 2047) calH4 -= 4096; // signed 12-bit
	calH5 = (ucCal[30] << 4) + (ucCal[29] >> 4);
	if (calH5 > 2047) calH5 -= 4096;
	calH6 = ucCal[31];
	if (calH6 > 127) calH6 -= 256; // signed char

	ucTemp[0] = 0xf2; // control humidity register
	ucTemp[1] = 0x01; // humidity over sampling rate = 1
	rc = write(file_i2c, ucTemp, 2);

	ucTemp[0] = 0xf4; // control measurement register
	ucTemp[1] = 0x27; // normal mode, temp and pressure over sampling rate=1
	rc = write(file_i2c, ucTemp, 2);

	ucTemp[0] = 0xf5; // CONFIG
	ucTemp[1] = 0xa0; // set stand by time to 1 second
	rc = write(file_i2c, ucTemp, 2);
	if (rc < 0) {} // suppress warning

	return 0;
} 

int bme280ReadValues(int *T, int *P, int *H)
{
    unsigned char ucTemp[16];
    int i,rc;
    int t, p, h; // raw sensor values
    int var1,var2,t_fine;
    int64_t P_64;
    int64_t var1_64, var2_64;

	ucTemp[0] = 0xf7; // start of data registers we want
	rc = write(file_i2c, ucTemp, 1); // write address of register to read
	i = read(file_i2c, ucTemp, 8);
	if (rc < 0 || i != 8)
	{
		return -1; // something went wrong
	}
	p = (ucTemp[0] << 12) + (ucTemp[1] << 4) + (ucTemp[2] >> 4);
	t = (ucTemp[3] << 12) + (ucTemp[4] << 4) + (ucTemp[5] >> 4);
	h = (ucTemp[6] << 8) + ucTemp[7];
    //	printf("raw values: p = %d, t = %d, h = %d\n", p, t, h);
	// Calculate calibrated temperature value
	// the value is 100x C (e.g. 2601 = 26.01C)
	var1 = ((((t >> 3) - (calT1 <<1))) * (calT2)) >> 11;
	var2 = (((((t >> 4) - (calT1)) * ((t>>4) - (calT1))) >> 12) * (calT3)) >> 14;
	t_fine = var1 + var2;
	*T = (t_fine * 5 + 128) >> 8;

	// Calculate calibrated pressure value
	var1_64 = t_fine - 128000;
	var2_64 = var1_64 * var1_64 * (int64_t)calP6;
	var2_64 = var2_64 + ((var1_64 * (int64_t)calP5) << 17);
	var2_64 = var2_64 + (((int64_t)calP4) << 35);
	var1_64 = ((var1_64 * var1_64 * (int64_t)calP3)>>8) + ((var1_64 * (int64_t)calP2)<<12);
	var1_64 = (((((int64_t)1)<<47)+var1_64))*((int64_t)calP1)>>33;
	if (var1_64 == 0) {
        *P = 0;
    } else {
        P_64 = 1048576 - p;
        P_64 = (((P_64<<31)-var2_64)*3125)/var1_64;
        var1_64 = (((int64_t)calP9) * (P_64>>13) * (P_64>>13)) >> 25;
        var2_64 = (((int64_t)calP8) * P_64) >> 19;
        P_64 = ((P_64 + var1_64 + var2_64) >> 8) + (((int64_t)calP7)<<4);
        *P = (int)P_64 / 100;
    }

	// Calculate calibrated humidity value
	var1 = (t_fine - 76800);
	var1 = (((((h << 14) - ((calH4) << 20) - ((calH5) * var1)) + 
		(16384)) >> 15) * (((((((var1 * (calH6)) >> 10) * (((var1 * (calH3)) >> 11) + (32768))) >> 10) + (2097152)) * (calH2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * (calH1)) >> 4));
	var1 = (var1 < 0? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);
	*H = var1 >> 12;
	return 0;
} 

// Six POSIX queues for ordered hand-offs:
//
//   TS → OPAO   "/status_req_queue"
//   TS → HAO    "/poll_req_queue"
//   OPAO → HAO  "/status_req_opao2hao"
//   HAO → OPAO  "/status_res_hao2opao"
//   HAO → BAO   "/poll_req_hao2bao"
//   BAO → HAO   "/poll_req_hao2bao_ack"
#define Q_TS_OPAO    "/status_req_queue"
#define Q_TS_HAO     "/poll_req_queue"
#define Q_OPAO_HAO   "/status_req_opao2hao"
#define Q_HAO_OPAO   "/status_res_hao2opao"
#define Q_HAO_BAO    "/poll_req_hao2bao"
#define Q_BAO_HAO    "/poll_req_hao2bao_ack"

#define STATUS_REQ   "STATUS_REQ"
#define POLL_REQ     "POLL_REQ"
#define STATUS_RES   "STATUS_RES"
#define POLL_RES     "POLL_RES"

#define MSG_SIZE     64
#define INTERVAL_SEC 1

static mqd_t mq_ts_opao  = (mqd_t)-1;
static mqd_t mq_ts_hao   = (mqd_t)-1;
static mqd_t mq_opao_hao = (mqd_t)-1;
static mqd_t mq_hao_opao = (mqd_t)-1;
static mqd_t mq_hao_bao  = (mqd_t)-1;
static mqd_t mq_bao_hao  = (mqd_t)-1;

static int hao_counter = 0;
static int last_bao_value = 0;  // <-- store the latest BAO number
static int fd_ts_opao;
static int fd_ts_hao;
static int fd_opao_hao;
static int fd_hao_opao;
static int fd_hao_bao;
static int fd_bao_hao;

void cleanup_and_exit(int signo) {
    if (mq_ts_opao  != (mqd_t)-1) { mq_close(mq_ts_opao);  mq_unlink(Q_TS_OPAO); }
    if (mq_ts_hao   != (mqd_t)-1) { mq_close(mq_ts_hao);   mq_unlink(Q_TS_HAO);  }
    if (mq_opao_hao != (mqd_t)-1) { mq_close(mq_opao_hao); mq_unlink(Q_OPAO_HAO);}
    if (mq_hao_opao != (mqd_t)-1) { mq_close(mq_hao_opao); mq_unlink(Q_HAO_OPAO);}
    if (mq_hao_bao  != (mqd_t)-1) { mq_close(mq_hao_bao);  mq_unlink(Q_HAO_BAO); }
    if (mq_bao_hao  != (mqd_t)-1) { mq_close(mq_bao_hao);  mq_unlink(Q_BAO_HAO); }
    if (fd_ts_opao  >= 0) close(fd_ts_opao);
    if (fd_ts_hao   >= 0) close(fd_ts_hao);
    if (fd_opao_hao >= 0) close(fd_opao_hao);
    if (fd_hao_opao >= 0) close(fd_hao_opao);
    if (fd_hao_bao  >= 0) close(fd_hao_bao);
    if (fd_bao_hao  >= 0) close(fd_bao_hao);
    printf("\nCleaned up on Ctrl-C\n");
    exit(EXIT_SUCCESS);
}

// TS: sends POLL_REQ to HAO twice as fast as STATUS_REQ to OPAO
void *time_sequencer(void *arg) {
    (void)arg;
    struct timespec ts      = { INTERVAL_SEC, 0 };
    struct timespec ts_half = { 0, INTERVAL_SEC * 1000000000L / 2 };
    struct timespec now;

    while (1) {
        /* First POLL_REQ after half interval */
        nanosleep(&ts_half, NULL);
        mq_send(mq_ts_hao, POLL_REQ, strlen(POLL_REQ) + 1, 0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("[TS]: [%5ld.%09ld] Sent to HAO: %s\n",
               now.tv_sec, now.tv_nsec, POLL_REQ);
        fflush(stdout);

        /* Second POLL_REQ after another half interval */
        nanosleep(&ts_half, NULL);
        mq_send(mq_ts_hao, POLL_REQ, strlen(POLL_REQ) + 1, 0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("[TS]: [%5ld.%09ld] Sent to HAO: %s\n",
               now.tv_sec, now.tv_nsec, POLL_REQ);
        fflush(stdout);

        /* Then STATUS_REQ after a full interval */
        nanosleep(&ts, NULL);
        mq_send(mq_ts_opao, STATUS_REQ, strlen(STATUS_REQ) + 1, 0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("[TS]: [%5ld.%09ld] Sent to OPAO: %s\n",
               now.tv_sec, now.tv_nsec, STATUS_REQ);
        fflush(stdout);
    }
    return NULL;
}

// OPAO: forwards STATUS_REQ TS→OPAO → OPAO→HAO, prints STATUS_RES:<n>
void *opao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE];
    struct timespec now;
    ssize_t len;
    struct pollfd pfds[2] = {
        { .fd = fd_ts_opao,  .events = POLLIN },
        { .fd = fd_hao_opao, .events = POLLIN }
    };

    while (1) {
        if (poll(pfds, 2, -1) == -1) { perror("[OPAO]: poll"); break; }

        if (pfds[0].revents & POLLIN) {
            len = mq_receive(mq_ts_opao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[OPAO]: mq_receive TS→OPAO"); break; }
            if (strcmp(buf, STATUS_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[OPAO]: [%5ld.%09ld] From TS: %s\n",
                       now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);
                mq_send(mq_opao_hao, buf, len, 0);
            }
        }

        if (pfds[1].revents & POLLIN) {
            len = mq_receive(mq_hao_opao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[OPAO]: mq_receive HAO→OPAO"); break; }
            if (strncmp(buf, STATUS_RES ":", strlen(STATUS_RES)+1) == 0) {
                printf("----------------------------------------------------\n");
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[OPAO]: [%5ld.%09ld] %s From HAO\n",
                       now.tv_sec, now.tv_nsec, buf);
                printf("----------------------------------------------------\n");
                fflush(stdout);
            }
        }
    }
    return NULL;
}

// HAO: handles POLL_REQ (forward to BAO), POLL_RES (store),
//      STATUS_REQ (reply using last_bao_value)
void *hao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE];
    struct timespec now;
    ssize_t len;
    struct pollfd pfds[3] = {
        { .fd = fd_ts_hao,   .events = POLLIN },
        { .fd = fd_opao_hao, .events = POLLIN },
        { .fd = fd_bao_hao,  .events = POLLIN }
    };

    while (1) {
        if (poll(pfds, 3, -1) == -1) { perror("[HAO]: poll"); break; }

        // TS → HAO: POLL_REQ
        if (pfds[0].revents & POLLIN) {
            len = mq_receive(mq_ts_hao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[HAO]: mq_receive TS→HAO"); break; }
            if (strcmp(buf, POLL_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[HAO]: [%5ld.%09ld] From TS: %s\n",
                       now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);

                // forward to BAO
                if (mq_send(mq_hao_bao, buf, len, 0) == -1) {
                    perror("[HAO]: mq_send HAO→BAO"); break;
                }
            }
        }

        // OPAO → HAO: STATUS_REQ
        if (pfds[1].revents & POLLIN) {
            len = mq_receive(mq_opao_hao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[HAO]: mq_receive OPAO→HAO"); break; }
            if (strcmp(buf, STATUS_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[HAO]: [%5ld.%09ld] From OPAO: %s\n",
                       now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);
                // reply using last_bao_value
                char resp[MSG_SIZE];
                snprintf(resp, MSG_SIZE, STATUS_RES ":%d", last_bao_value);
                if (mq_send(mq_hao_opao, resp, strlen(resp)+1, 0) == -1) {
                    perror("[HAO]: mq_send HAO→OPAO"); break;
                }
            }
        }

        // BAO → HAO: POLL_RES:<n>
        if (pfds[2].revents & POLLIN) {
            len = mq_receive(mq_bao_hao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[HAO]: mq_receive BAO→HAO"); break; }
            if (strncmp(buf, POLL_RES ":", strlen(POLL_RES)+1) == 0) {
                // parse and store the <n>
                last_bao_value = atoi(buf + strlen(POLL_RES) + 1);
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[HAO]: [%5ld.%09ld] From BAO: %s\n",
                       now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);
            }
        }
    }
    return NULL;
}

// BAO: listens for POLL_REQ forwarded by HAO → BAO,
//      then replies with POLL_RES:<n>
void *bao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE];
    struct timespec now;
    ssize_t len;
    int bao_counter = 0;
    struct pollfd p_fb = { .fd = fd_hao_bao, .events = POLLIN };

    int T, P, H; // calibrated values

    if (bme280Init(1, 0x76) != 0) 
    {
        fprintf(stderr, "[BAO]: BME280 init error\n");
    }
    
    while (1) {
        if (poll(&p_fb, 1, -1) == -1) { perror("[BAO]: poll"); break; }
        if (p_fb.revents & POLLIN) {
            len = mq_receive(mq_hao_bao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[BAO]: mq_receive HAO→BAO"); break; }
            if (strcmp(buf, POLL_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[BAO]: [%5ld.%09ld] From HAO: %s\n",
                       now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);
                
                if (bme280ReadValues(&T, &P, &H) == 0) {
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    printf("[BAO]: [%5ld.%09ld] Sensor → T=%d°C  P=%dPa  H=%d%%\n",
                        now.tv_sec, now.tv_nsec, T, P, H);
                } else {
                    fprintf(stderr, "[BAO]: Error reading BME280\n");
                }

                // send POLL_RES:<n>
                char ack[MSG_SIZE];
                snprintf(ack, MSG_SIZE, POLL_RES ":%d", ++bao_counter);
                if (mq_send(mq_bao_hao, ack, strlen(ack)+1, 0) == -1) {
                    perror("[BAO]: mq_send BAO→HAO"); break;
                }
            }
        }
    }
    return NULL;
}

int main(void) {
    struct mq_attr attr = {
        .mq_flags   = 0,
        .mq_maxmsg  = 10,
        .mq_msgsize = MSG_SIZE,
        .mq_curmsgs = 0
    };
    pthread_t ts_tid, op_tid, ha_tid, bao_tid;
    signal(SIGINT, cleanup_and_exit);

    /* create all six queues */
    mq_ts_opao  = mq_open(Q_TS_OPAO,  O_CREAT|O_RDWR, 0666, &attr);
    mq_ts_hao   = mq_open(Q_TS_HAO,   O_CREAT|O_RDWR, 0666, &attr);
    mq_opao_hao = mq_open(Q_OPAO_HAO, O_CREAT|O_RDWR, 0666, &attr);
    mq_hao_opao = mq_open(Q_HAO_OPAO, O_CREAT|O_RDWR, 0666, &attr);
    mq_hao_bao  = mq_open(Q_HAO_BAO,  O_CREAT|O_RDWR, 0666, &attr);
    mq_bao_hao  = mq_open(Q_BAO_HAO,  O_CREAT|O_RDWR, 0666, &attr);

    fd_ts_opao  = open("/dev/mqueue" Q_TS_OPAO,  O_RDONLY);
    fd_ts_hao   = open("/dev/mqueue" Q_TS_HAO,   O_RDONLY);
    fd_opao_hao = open("/dev/mqueue" Q_OPAO_HAO, O_RDONLY);
    fd_hao_opao = open("/dev/mqueue" Q_HAO_OPAO, O_RDONLY);
    fd_hao_bao  = open("/dev/mqueue" Q_HAO_BAO,  O_RDONLY);
    fd_bao_hao  = open("/dev/mqueue" Q_BAO_HAO,  O_RDONLY);

    /* spawn threads */
    pthread_create(&ts_tid,  NULL, time_sequencer, NULL);
    pthread_create(&op_tid,  NULL, opao,          NULL);
    pthread_create(&ha_tid,  NULL, hao,           NULL);
    pthread_create(&bao_tid, NULL, bao,           NULL);

    /* wait */
    pthread_join(ts_tid,   NULL);
    pthread_join(op_tid,   NULL);
    pthread_join(ha_tid,   NULL);
    pthread_join(bao_tid,  NULL);

    cleanup_and_exit(0);
    return 0;
}
