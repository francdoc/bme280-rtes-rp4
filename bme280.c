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

int bme280Init(int iChannel, int iAddr)
{
    int rc, nbytes;
    unsigned char ucTemp[32];
    unsigned char ucCal[36];
    char filename[32];

    // Build I²C bus path and open it
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", iChannel);
    file_i2c = open(filename, O_RDWR);
    if (file_i2c < 0) {
        perror("bme280Init: open i2c bus");
        return -1;
    }

    // Point at the correct slave address
    if (ioctl(file_i2c, I2C_SLAVE, iAddr) < 0) {
        perror("bme280Init: ioctl I2C_SLAVE");
        close(file_i2c);
        return -1;
    }

    // Read and verify chip ID (0xD0 -> 0x60)
    ucTemp[0] = 0xD0;
    rc = write(file_i2c, ucTemp, 1);
    if (rc < 0) {
        perror("bme280Init: write ID register");
        close(file_i2c);
        return -1;
    }

    nbytes = read(file_i2c, ucTemp, 1);
    if (nbytes != 1) {
        perror("bme280Init: read ID register");
        close(file_i2c);
        return -1;
    }
    if (ucTemp[0] != 0x60) {
        fprintf(stderr, "bme280Init: wrong ID 0x%02X, expected 0x60\n", ucTemp[0]);
        close(file_i2c);
        return -1;
    }

    // Read 24 bytes of calibration data starting at 0x88
    ucTemp[0] = 0x88;
    rc = write(file_i2c, ucTemp, 1);
    if (rc < 0) {
        perror("bme280Init: write calib start");
        close(file_i2c);
        return -1;
    }

    nbytes = read(file_i2c, ucCal, 24);
    if (nbytes != 24) {
        perror("bme280Init: read calib block");
        close(file_i2c);
        return -1;
    }

    // Read humidity calibration byte at 0xA1
    ucTemp[0] = 0xA1;
    write(file_i2c, ucTemp, 1);
    read (file_i2c, &ucCal[24], 1);

    // Read remaining humidity calibration bytes at 0xE1
    ucTemp[0] = 0xE1;
    write(file_i2c, ucTemp, 1);
    read (file_i2c, &ucCal[25], 7);

    calT1 = ucCal[0]  | (ucCal[1] << 8);
    calT2 = ucCal[2]  | (ucCal[3] << 8);  if (calT2 > 32767) calT2 -= 65536;
    calT3 = ucCal[4]  | (ucCal[5] << 8);  if (calT3 > 32767) calT3 -= 65536;

    calP1 = ucCal[6]  | (ucCal[7] << 8);
    calP2 = ucCal[8]  | (ucCal[9] << 8);  if (calP2 > 32767) calP2 -= 65536;
    calP3 = ucCal[10] | (ucCal[11]<< 8);  if (calP3 > 32767) calP3 -= 65536;
    calP4 = ucCal[12] | (ucCal[13]<< 8);  if (calP4 > 32767) calP4 -= 65536;
    calP5 = ucCal[14] | (ucCal[15]<< 8);  if (calP5 > 32767) calP5 -= 65536;
    calP6 = ucCal[16] | (ucCal[17]<< 8);  if (calP6 > 32767) calP6 -= 65536;
    calP7 = ucCal[18] | (ucCal[19]<< 8);  if (calP7 > 32767) calP7 -= 65536;
    calP8 = ucCal[20] | (ucCal[21]<< 8);  if (calP8 > 32767) calP8 -= 65536;
    calP9 = ucCal[22] | (ucCal[23]<< 8);  if (calP9 > 32767) calP9 -= 65536;

    calH1 = ucCal[24];
    calH2 = ucCal[25] | (ucCal[26]<< 8);  if (calH2 > 32767) calH2 -= 65536;
    calH3 = ucCal[27];
    calH4 = (ucCal[28]<<4)  | (ucCal[29] & 0x0F);  if (calH4 > 2047) calH4 -= 4096;
    calH5 = (ucCal[30]<<4)  | (ucCal[29] >> 4);    if (calH5 > 2047) calH5 -= 4096;
    calH6 = ucCal[31];       if (calH6 > 127)    calH6 -= 256;

    // Finally configure the sensor as before
    ucTemp[0] = 0xF2; ucTemp[1] = 0x01; write(file_i2c, ucTemp, 2);
    ucTemp[0] = 0xF4; ucTemp[1] = 0x27; write(file_i2c, ucTemp, 2);
    ucTemp[0] = 0xF5; ucTemp[1] = 0xA0; write(file_i2c, ucTemp, 2);

    return 0;
}

#define Q_TS_BAO     "/poll_req_queue"          // TS → BAO
#define Q_BAO_HAO    "/poll_req_hao2bao_ack"    // BAO → HAO

#define POLL_REQ     "POLL_REQ"
#define POLL_RES     "POLL_RES"
#define MSG_SIZE     64
#define INTERVAL_SEC 1

static mqd_t mq_ts_bao  = (mqd_t)-1;
static mqd_t mq_bao_hao = (mqd_t)-1;

void cleanup_and_exit(int signo) {
    if (mq_ts_bao  != (mqd_t)-1) { mq_close(mq_ts_bao);  mq_unlink(Q_TS_BAO); }
    if (mq_bao_hao != (mqd_t)-1) { mq_close(mq_bao_hao); mq_unlink(Q_BAO_HAO); }
    printf("\nCleaned up on Ctrl-C\n");
    exit(EXIT_SUCCESS);
}

// TS: sends POLL_REQ into BAO’s queue
void *time_sequencer(void *arg) {
    (void)arg;
    struct timespec ts_half = { 0, INTERVAL_SEC * 1000000000L / 2 };
    struct timespec now;

    while (1) {
        nanosleep(&ts_half, NULL);
        mq_send(mq_ts_bao, POLL_REQ, strlen(POLL_REQ) + 1, 0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("[TS]: [%5ld.%09ld] Sent to BAO: %s\n",
               now.tv_sec, now.tv_nsec, POLL_REQ);
        fflush(stdout);
    }
    return NULL;
}

// BAO: listens on mq_ts_bao for POLL_REQ → do I²C → reply on mq_bao_hao
void *bao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE+1];
    struct timespec now;
    ssize_t len;
    int bao_counter = 0;
    struct pollfd p_fd = { .fd = (int)mq_ts_bao, .events = POLLIN };
    unsigned char data[8];

    while (1) {
        if (poll(&p_fd, 1, -1) == -1) { perror("[BAO]: poll"); break; }
        if (p_fd.revents & POLLIN) {
            len = mq_receive(mq_ts_bao, buf, MSG_SIZE+1, NULL);
            if (len < 0) { perror("[BAO]: mq_receive"); break; }

            if (strcmp(buf, POLL_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[BAO]: [%5ld.%09ld] From TS: %s\n",
                       now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);

                // raw I2C read
                uint8_t cmd = 0xF7;
                write(file_i2c, &cmd, 1);
                int rd = read(file_i2c, data, 8);

                if (rd == 8) {
                    printf("[BAO]: read %d bytes:", rd);
                    for (int i = 0; i < rd; i++) printf(" %02X", data[i]);
                    printf("\n");
                } else {
                    printf("[BAO]: I2C read error, got %d bytes\n", rd);
                }

                // build and send reply
                char hdr[MSG_SIZE];
                int hlen = snprintf(hdr, MSG_SIZE, POLL_RES ":%d", ++bao_counter) + 1;
                unsigned char msg[MSG_SIZE + 8];
                memcpy(msg, hdr, hlen);
                memcpy(msg + hlen, data, 8);

                mq_send(mq_bao_hao, (char*)msg, hlen + 8, 0);
            }
        }
    }
    return NULL;
}

int main(void) {
    signal(SIGINT, cleanup_and_exit);

    // init BME280
    if (bme280Init(1, 0x76) < 0) {
        printf("[MAIN]: failed to init BME280 sensor.\n");
    };

    // prepare queues
    struct mq_attr attr = { .mq_flags=0, .mq_maxmsg=10, .mq_msgsize=MSG_SIZE, .mq_curmsgs=0 };
    struct mq_attr attr_raw = attr; attr_raw.mq_msgsize = MSG_SIZE + 8;

    mq_ts_bao  = mq_open(Q_TS_BAO,  O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_bao_hao = mq_open(Q_BAO_HAO, O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr_raw);
    if (mq_ts_bao == (mqd_t)-1 || mq_bao_hao == (mqd_t)-1) {
        perror("mq_open");
        exit(EXIT_FAILURE);
    }

    // spawn threads
    pthread_t ts_tid, bao_tid;
    pthread_create(&ts_tid,  NULL, time_sequencer, NULL);
    pthread_create(&bao_tid, NULL, bao,           NULL);

    // wait
    pthread_join(ts_tid,  NULL);
    pthread_join(bao_tid, NULL);

    cleanup_and_exit(0);
    return 0;
}