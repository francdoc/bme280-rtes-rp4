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

// Six POSIX queues for ordered hand-offs:
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
static int last_bao_value = 0; 

void decode_bme280_readout(unsigned char *raw, int cnt, int *outT, int *outP, int *outH);

void decode_bme280_readout(unsigned char *raw, int cnt, int *outT, int *outP, int *outH)
{
    // decode big-endian raw[0..2] into p, raw[3..5] into t, raw[6..7] into h
    int p = (raw[0]<<12) | (raw[1]<<4) | (raw[2]>>4);
    int t = (raw[3]<<12) | (raw[4]<<4) | (raw[5]>>4);
    int h = (raw[6]<<8)  |  raw[7];

    // now run your existing calibration math...
    int var1 = ((((t >> 3) - (calT1 <<1))) * calT2) >> 11;
    int var2 = (((((t >> 4) - calT1) * ((t>>4) - calT1)) >> 12) * calT3) >> 14;
    int t_fine = var1 + var2;
    int TT = (t_fine * 5 + 128) >> 8;

    int64_t v1_64 = t_fine - 128000;
    int64_t v2_64 = v1_64 * v1_64 * calP6;
    v2_64 += (v1_64 * calP5) << 17;
    v2_64 += ((int64_t)calP4) << 35;
    v1_64  = ((v1_64*v1_64*calP3)>>8) + ((v1_64*calP2)<<12);
    v1_64  = (((((int64_t)1)<<47)+v1_64)*calP1)>>33;
    int PP = 0;
    
    if (v1_64) {
        int64_t P_64 = 1048576 - p;
        P_64 = (((P_64<<31)-v2_64)*3125)/v1_64;
        v1_64 = (calP9 * (P_64>>13) * (P_64>>13)) >> 25;
        v2_64 = (calP8 * P_64) >> 19;
        P_64  = ((P_64 + v1_64 + v2_64)>>8) + ((int64_t)calP7<<4);
        PP = P_64/100;
    }

    int varH = t_fine - 76800;
    varH = (((((h<<14) - (calH4<<20) - (calH5*varH)) + 16384)>>15) *
        (((((varH*calH6)>>10) * (((varH*calH3)>>11) + 32768))>>10) + 2097152) *
        calH2 + 8192) >> 14;
    varH = varH - (((((varH>>15)*(varH>>15))>>7)*calH1)>>4);
    if (varH<0) varH=0;
    if (varH>419430400) varH=419430400;
    int HH = varH>>12;

    // store for STATUS_RES replies
    last_bao_value = cnt;

    *outT = TT; // °C
    *outP = PP; // Pa
    *outH = HH; // %
}

void cleanup_and_exit(int signo) {
    if (mq_ts_opao  != (mqd_t)-1) { mq_close(mq_ts_opao);  mq_unlink(Q_TS_OPAO); }
    if (mq_ts_hao   != (mqd_t)-1) { mq_close(mq_ts_hao);   mq_unlink(Q_TS_HAO);  }
    if (mq_opao_hao != (mqd_t)-1) { mq_close(mq_opao_hao); mq_unlink(Q_OPAO_HAO);}
    if (mq_hao_opao != (mqd_t)-1) { mq_close(mq_hao_opao); mq_unlink(Q_HAO_OPAO);}
    if (mq_hao_bao  != (mqd_t)-1) { mq_close(mq_hao_bao);  mq_unlink(Q_HAO_BAO); }
    if (mq_bao_hao  != (mqd_t)-1) { mq_close(mq_bao_hao);  mq_unlink(Q_BAO_HAO); }
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
        { .fd = (int)mq_ts_opao,  .events = POLLIN },
        { .fd = (int)mq_hao_opao, .events = POLLIN }
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

// HAO: handles POLL_REQ (forward to BAO), POLL_RES+raw (store & decode),
//      STATUS_REQ (reply using last_bao_value)
void *hao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE + 8];// allow space for 8 raw bytes
    struct timespec now;
    ssize_t len;

    struct pollfd pfds[3] = {
        { .fd = (int)mq_ts_hao,   .events = POLLIN },
        { .fd = (int)mq_opao_hao, .events = POLLIN },
        { .fd = (int)mq_bao_hao,  .events = POLLIN }
    };

    int last_T = 0, last_P = 0, last_H = 0;

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

                // build "POLL_REQ\0" + 0xF7
                unsigned char msg[MSG_SIZE+1];
                memcpy(msg, POLL_REQ, strlen(POLL_REQ)+1); // include '\0'
                msg[strlen(POLL_REQ)+1] = 0xF7; // raw I²C command

                // print the raw I²C command byte we will send (0xF7)
                printf("[HAO]: sending raw I2C cmd -> 0x%02X to BAO\n",
                msg[strlen(POLL_REQ)+1]);
                mq_send(mq_hao_bao, (char*)msg, strlen(POLL_REQ)+2, 0);
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
                char resp[MSG_SIZE];
                snprintf(resp, MSG_SIZE,
                         STATUS_RES ":%d T=%d P=%d H=%d",
                         last_bao_value, last_T, last_P, last_H);
                mq_send(mq_hao_opao, resp, strlen(resp)+1, 0);
            }
        }

        // BAO → HAO: POLL_RES:<n>\0 + 8 raw bytes
        if (pfds[2].revents & POLLIN) {
            len = mq_receive(mq_bao_hao, buf, MSG_SIZE+8, NULL);
            if (len == -1) { perror("[HAO]: mq_receive BAO→HAO"); break; }
            
            if (strncmp(buf, POLL_RES ":", strlen(POLL_RES) + 1) == 0) {
                int got = len;  // length returned by mq_receive
                fprintf(stderr, "[HAO]: Received from BAO → ");
                for (int i = 0; i < got; i++) {
                    fprintf(stderr, "%02X ", (unsigned char)buf[i]);
                }
                fprintf(stderr, "\n");

                // split header and raw
                char *hdr = buf; // "POLL_RES:<n>\0"
                unsigned char *raw = (unsigned char*)(buf + strlen(hdr) + 1);
                int cnt = atoi(hdr + strlen(POLL_RES) + 1);
                
                decode_bme280_readout(raw, cnt, &last_T, &last_P, &last_H);

                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[HAO]: [%5ld.%09ld] From BAO: count=%d, T=%d°C, P=%dPa, H=%d%%\n",
                    now.tv_sec, now.tv_nsec, cnt, last_T, last_P, last_H);
                fflush(stdout);
            }
        }
    }
    return NULL;
}

// BAO: listens for POLL_REQ\0+cmd → I²C transaction → reply POLL_RES:<n>\0 + 8 raw bytes
void *bao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE+1];
    struct timespec now;
    ssize_t len;
    int bao_counter = 0;
    struct pollfd p_fb = { .fd = (int)mq_hao_bao, .events = POLLIN };
    
    unsigned char data[8];
    while (1) {
        if (poll(&p_fb, 1, -1) == -1) { perror("[BAO]: poll"); break; }
        if (p_fb.revents & POLLIN) {
            len = mq_receive(mq_hao_bao, buf, MSG_SIZE+1, NULL);
            if (len == -1) { perror("[BAO]: mq_receive HAO→BAO"); break; }
            
            if (strcmp(buf, POLL_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[BAO]: [%5ld.%09ld] From HAO: %s\n",
                       now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);

                // extract raw command byte
                uint8_t cmd = (uint8_t)buf[strlen(POLL_REQ)+1];

                // print the raw I²C command byte we just received (should be 0xF7)
                printf("[BAO]: received raw I2C cmd -> 0x%02X from HAO\n", cmd);

                // do just the I²C
                write(file_i2c, &cmd, 1);
                int rd = read(file_i2c, data, 8);

                // -- log after the I2C transaction --
                if (rd == 8) {
                    printf("[BAO]: read %d raw bytes:", rd);
                    for (int i = 0; i < 8; i++) printf(" %02X", data[i]);
                    printf("\n");
                } else {
                    printf("[BAO]: I2C read error, expected 8 bytes, got %d\n", rd);
                }
                // build reply: "POLL_RES:<n>\0" + 8 bytes
                char hdr[MSG_SIZE];
                int hlen = snprintf(hdr, MSG_SIZE, POLL_RES ":%d", ++bao_counter) + 1;
                unsigned char msg[hlen + 8];
                memcpy(msg, hdr, hlen);
                memcpy(msg + hlen, data, 8);

                // print out the header + raw bytes in hex
                int total = hlen + 8;
                fprintf(stderr, "[BAO]: Sending to HAO → ");
                for (int i = 0; i < total; i++) {
                    fprintf(stderr, "%02X ", ((unsigned char*)msg)[i]);
                }
                fprintf(stderr, "\n");
            
                mq_send(mq_bao_hao, (char*)msg, hlen + 8, 0);
            }
        }
    }
    return NULL;
}

// #define RETRY_INIT

int main(void) {
    #ifdef RETRY_INIT
        while (1) {
            if (bme280Init(1, 0x76) == 0) {
                printf("BME280 initialized successfully\n");
                break;
            }
            fprintf(stderr, "BME280 init error, retrying in 1s...\n");
            sleep(1);
        }
    #endif

    #ifndef  RETRY_INIT
        if (bme280Init(1, 0x76) == 0) {
            printf("BME280 initialized successfully\n");
            return 0;
        }
    #endif

    pthread_t ts_tid, op_tid, ha_tid, bao_tid;
    signal(SIGINT, cleanup_and_exit);

    struct mq_attr attr = {
        .mq_flags   = 0,
        .mq_maxmsg  = 10,
        .mq_msgsize = MSG_SIZE,
        .mq_curmsgs = 0
    };
    struct mq_attr attr_raw = attr;
    attr_raw.mq_msgsize = MSG_SIZE + 8;  // BAO→HAO carries header + 8 raw bytes

    /* create all six queues (nonblocking) */
    mq_ts_opao  = mq_open(Q_TS_OPAO,  O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_ts_hao   = mq_open(Q_TS_HAO,   O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_opao_hao = mq_open(Q_OPAO_HAO, O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_hao_opao = mq_open(Q_HAO_OPAO, O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_hao_bao  = mq_open(Q_HAO_BAO,  O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_bao_hao  = mq_open(Q_BAO_HAO,  O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr_raw);

    if (mq_ts_opao==(mqd_t)-1 || mq_ts_hao==(mqd_t)-1 || mq_opao_hao==(mqd_t)-1 ||
        mq_hao_opao==(mqd_t)-1 || mq_hao_bao==(mqd_t)-1 || mq_bao_hao==(mqd_t)-1) {
        perror("mq_open");
        exit(EXIT_FAILURE);
    }

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