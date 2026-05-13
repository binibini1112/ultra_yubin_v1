#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#define REG_CTRL        0x00u
#define REG_PAN_GOAL    0x04u
#define REG_TILT_GOAL   0x08u
#define REG_IDS         0x0cu
#define REG_STATUS      0x10u
#define REG_LAST_PAN    0x14u
#define REG_LAST_TILT   0x18u
#define REG_TRACK_BOX   0x30u
#define REG_TRACK_XY    0x34u
#define REG_TRACK_FRAME 0x38u
#define REG_TRACK_CMD   0x3cu

#define TRACK_CMD_VALID 0x00000001u
#define TRACK_CMD_TRACK 0x00000002u

#define MAP_SIZE 0x1000u
#define PAN_CENTER 2048u
#define TILT_CENTER 2772u
#define GOAL_MIN 0u
#define GOAL_MAX 4095u
#define AUDIO_TICKS_PER_DEG 11
#define ADDR_TORQUE_ENABLE 64u
#define ADDR_PROFILE_ACCEL 108u
#define ADDR_PROFILE_VELOCITY 112u
#define ADDR_GOAL_POSITION 116u
#define ADDR_PRESENT_POSITION 132u
#define TRACK_DEADBAND_X 18
#define TRACK_DEADBAND_Y 16
#define TRACK_TICK_PER_PIXEL_NUM 1
#define TRACK_TICK_PER_PIXEL_DEN 2
#define TRACK_MAX_CORRECTION 900
#define TRACK_PAN_LIMIT_FROM_FRONT 900
#define TRACK_TILT_LIMIT_FROM_FRONT 700
#define PL_CMD_SET_PAN  0x10000000u
#define PL_CMD_SET_TILT 0x20000000u
#define PL_CMD_SET_CX   0x30000000u
#define PL_CMD_SET_CY   0x40000000u
#define PL_CMD_SET_FW   0x50000000u
#define PL_CMD_SET_FH   0x60000000u
#define PL_CMD_TRACK    0x70000000u
#define PL_CMD_SET_BOX  0x80000000u

static volatile sig_atomic_t g_running = 1;

static void on_signal(int signo)
{
    (void)signo;
    g_running = 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s --base 0xA0000000 [--port 5016] [--serial /dev/ttyUSB0] "
        "[--baud 57600] [--pan-id 1] [--tilt-id 2] [--dry-run] [--no-pl] "
        "[--skip-pl-init] [--skip-dxl-init] [--lazy-pl-open] [--profile-accel 90] [--profile-velocity 280] "
        "[--center-file /home/xilinx/ultra_yubin/front_center.env]\n",
        argv0);
}

static uint32_t parse_u32(const char *s)
{
    return (uint32_t)strtoul(s, NULL, 0);
}

static uint32_t clamp_goal_i64(int64_t value)
{
    if (value < (int64_t)GOAL_MIN) {
        return GOAL_MIN;
    }
    if (value > (int64_t)GOAL_MAX) {
        return GOAL_MAX;
    }
    return (uint32_t)value;
}

static int64_t clamp_correction_i64(int64_t value)
{
    if (value > TRACK_MAX_CORRECTION) {
        return TRACK_MAX_CORRECTION;
    }
    if (value < -TRACK_MAX_CORRECTION) {
        return -TRACK_MAX_CORRECTION;
    }
    return value;
}

static int goal_pair_valid(uint32_t pan_goal, uint32_t tilt_goal)
{
    if (pan_goal > GOAL_MAX || tilt_goal > GOAL_MAX) {
        return 0;
    }
    return !(pan_goal == 0u && tilt_goal == 0u);
}

static uint32_t clamp_around_front_i64(int64_t value, uint32_t front, int64_t limit)
{
    int64_t lo = (int64_t)front - limit;
    int64_t hi = (int64_t)front + limit;
    if (lo < (int64_t)GOAL_MIN) {
        lo = GOAL_MIN;
    }
    if (hi > (int64_t)GOAL_MAX) {
        hi = GOAL_MAX;
    }
    if (value < lo) {
        return (uint32_t)lo;
    }
    if (value > hi) {
        return (uint32_t)hi;
    }
    return (uint32_t)value;
}

static int goal_pair_near_front(uint32_t pan_goal, uint32_t tilt_goal,
                                uint32_t front_pan, uint32_t front_tilt)
{
    if (!goal_pair_valid(pan_goal, tilt_goal)) {
        return 0;
    }
    int64_t pan_delta = (int64_t)pan_goal - (int64_t)front_pan;
    int64_t tilt_delta = (int64_t)tilt_goal - (int64_t)front_tilt;
    if (pan_delta < -TRACK_PAN_LIMIT_FROM_FRONT || pan_delta > TRACK_PAN_LIMIT_FROM_FRONT) {
        return 0;
    }
    if (tilt_delta < -TRACK_TILT_LIMIT_FROM_FRONT || tilt_delta > TRACK_TILT_LIMIT_FROM_FRONT) {
        return 0;
    }
    return 1;
}

static void clamp_goal_pair_near_front(uint32_t *pan_goal, uint32_t *tilt_goal,
                                       uint32_t front_pan, uint32_t front_tilt)
{
    *pan_goal = clamp_around_front_i64((int64_t)*pan_goal, front_pan, TRACK_PAN_LIMIT_FROM_FRONT);
    *tilt_goal = clamp_around_front_i64((int64_t)*tilt_goal, front_tilt, TRACK_TILT_LIMIT_FROM_FRONT);
}

static void ps_track_step(uint32_t cx, uint32_t cy, uint32_t fw, uint32_t fh, uint32_t valid,
                          uint32_t front_pan, uint32_t front_tilt,
                          uint32_t *pan_goal, uint32_t *tilt_goal)
{
    if (!valid || fw == 0u || fh == 0u) {
        return;
    }

    int64_t err_x = (int64_t)cx - (int64_t)(fw / 2u);
    int64_t err_y = (int64_t)cy - (int64_t)(fh / 2u);
    int64_t pan_correction = 0;
    int64_t tilt_correction = 0;
    if (err_x > TRACK_DEADBAND_X || err_x < -TRACK_DEADBAND_X) {
        pan_correction = clamp_correction_i64(
            (err_x * TRACK_TICK_PER_PIXEL_NUM) / TRACK_TICK_PER_PIXEL_DEN);
    }
    if (err_y > TRACK_DEADBAND_Y || err_y < -TRACK_DEADBAND_Y) {
        tilt_correction = clamp_correction_i64(
            (err_y * TRACK_TICK_PER_PIXEL_NUM) / TRACK_TICK_PER_PIXEL_DEN);
    }

    *pan_goal = clamp_goal_i64((int64_t)front_pan + pan_correction);
    *tilt_goal = clamp_goal_i64((int64_t)front_tilt - tilt_correction);
}

static void reg_write(volatile uint32_t *regs, uint32_t offset, uint32_t value)
{
    regs[offset / 4u] = value;
}

static uint32_t reg_read(volatile uint32_t *regs, uint32_t offset)
{
    return regs[offset / 4u];
}

static void pl_ctrl_write(volatile uint32_t *regs, uint32_t value)
{
    __sync_synchronize();
    reg_write(regs, REG_CTRL, value);
    __sync_synchronize();
    (void)reg_read(regs, REG_CTRL);
    __sync_synchronize();
}

static uint32_t pl_read_pan_goal(volatile uint32_t *regs)
{
    return reg_read(regs, REG_CTRL);
}

static uint32_t pl_read_tilt_goal(volatile uint32_t *regs)
{
    return reg_read(regs, REG_STATUS);
}

static uint32_t pl_read_count(volatile uint32_t *regs)
{
    return reg_read(regs, REG_LAST_PAN);
}

static uint32_t pl_read_flags(volatile uint32_t *regs)
{
    return reg_read(regs, REG_LAST_TILT);
}

static void pl_cmd_set_goal(volatile uint32_t *regs, uint32_t pan, uint32_t tilt)
{
    pl_ctrl_write(regs, PL_CMD_SET_PAN | (pan & 0xfffu));
    pl_ctrl_write(regs, PL_CMD_SET_TILT | (tilt & 0xfffu));
}

static void pl_cmd_set_track(volatile uint32_t *regs, uint32_t cx, uint32_t cy,
                             uint32_t bw, uint32_t bh,
                             uint32_t fw, uint32_t fh, uint32_t conf, uint32_t valid)
{
    pl_ctrl_write(regs, PL_CMD_SET_CX | (cx & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_SET_CY | (cy & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_SET_BOX | ((bh & 0xfffu) << 16) | (bw & 0xfffu));
    pl_ctrl_write(regs, PL_CMD_SET_FW | (fw & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_SET_FH | (fh & 0xffffu));
    pl_ctrl_write(regs, PL_CMD_TRACK | ((conf & 0xffffu) << 8) |
                         (valid ? TRACK_CMD_VALID : 0u) | TRACK_CMD_TRACK);
}

static void pl_init_defaults(volatile uint32_t *regs, uint32_t pan_id, uint32_t tilt_id,
                             uint32_t front_pan, uint32_t front_tilt)
{
    pl_cmd_set_goal(regs, front_pan, front_tilt);
    pl_ctrl_write(regs, PL_CMD_SET_CX | 640u);
    pl_ctrl_write(regs, PL_CMD_SET_CY | 360u);
    pl_ctrl_write(regs, PL_CMD_SET_BOX | (60u << 16) | 80u);
    pl_ctrl_write(regs, PL_CMD_SET_FW | 1280u);
    pl_ctrl_write(regs, PL_CMD_SET_FH | 720u);
    reg_write(regs, REG_IDS, ((tilt_id & 0xffu) << 8) | (pan_id & 0xffu));
    pl_ctrl_write(regs, 1u);
}

static void load_center_file(const char *path, uint32_t *front_pan, uint32_t *front_tilt)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), fp)) {
        unsigned int value = 0;
        if (sscanf(line, "PAN=%u", &value) == 1 && value <= GOAL_MAX) {
            *front_pan = value;
        } else if (sscanf(line, "TILT=%u", &value) == 1 && value <= GOAL_MAX) {
            *front_tilt = value;
        }
    }
    fclose(fp);
}

static int save_center_file(const char *path, uint32_t front_pan, uint32_t front_tilt)
{
    FILE *fp = fopen(path, "w");
    if (!fp) {
        return -1;
    }
    fprintf(fp, "PAN=%u\nTILT=%u\n", front_pan, front_tilt);
    fclose(fp);
    return 0;
}

static int open_pl_regs(uint32_t base, int *mem_fd, void **map, volatile uint32_t **regs)
{
    if (*regs != NULL) {
        return 0;
    }

    *mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (*mem_fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    off_t page_base = (off_t)(base & ~(MAP_SIZE - 1u));
    off_t page_off = (off_t)(base - (uint32_t)page_base);
    *map = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, *mem_fd, page_base);
    if (*map == MAP_FAILED) {
        perror("mmap");
        close(*mem_fd);
        *mem_fd = -1;
        return -1;
    }

    *regs = (volatile uint32_t *)((uint8_t *)*map + page_off);
    return 0;
}

static void close_pl_regs(int *mem_fd, void **map, volatile uint32_t **regs)
{
    *regs = NULL;
    if (*map != MAP_FAILED) {
        munmap(*map, MAP_SIZE);
        *map = MAP_FAILED;
    }
    if (*mem_fd >= 0) {
        close(*mem_fd);
        *mem_fd = -1;
    }
}

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 57600: return B57600;
    case 115200: return B115200;
    case 1000000:
#ifdef B1000000
        return B1000000;
#else
        return B57600;
#endif
    default: return B57600;
    }
}

static int open_serial(const char *path, int baud)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static uint16_t dxl_crc_update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000u) {
            crc = (uint16_t)((crc << 1) ^ 0x8005u);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int write_all(int fd, const uint8_t *buf, size_t len)
{
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(fd, buf + off, len - off);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        off += (size_t)n;
    }
    tcdrain(fd);
    return 0;
}

static int dxl_write_data(int fd, uint8_t id, uint16_t addr,
                          const uint8_t *data, uint16_t data_len, int dry_run)
{
    uint8_t pkt[32];
    uint16_t param_len = (uint16_t)(2u + data_len);
    uint16_t pkt_len = (uint16_t)(param_len + 3u);
    size_t total_len = (size_t)(7u + pkt_len);
    if (total_len > sizeof(pkt)) {
        return -1;
    }

    pkt[0] = 0xff;
    pkt[1] = 0xff;
    pkt[2] = 0xfd;
    pkt[3] = 0x00;
    pkt[4] = id;
    pkt[5] = (uint8_t)(pkt_len & 0xffu);
    pkt[6] = (uint8_t)((pkt_len >> 8) & 0xffu);
    pkt[7] = 0x03;
    pkt[8] = (uint8_t)(addr & 0xffu);
    pkt[9] = (uint8_t)((addr >> 8) & 0xffu);
    for (uint16_t i = 0; i < data_len; i++) {
        pkt[10u + i] = data[i];
    }

    uint16_t crc = 0;
    for (size_t i = 0; i < total_len - 2u; i++) {
        crc = dxl_crc_update(crc, pkt[i]);
    }
    pkt[total_len - 2u] = (uint8_t)(crc & 0xffu);
    pkt[total_len - 1u] = (uint8_t)((crc >> 8) & 0xffu);

    if (dry_run) {
        return 0;
    }
    return write_all(fd, pkt, total_len);
}

static int dxl_read_data(int fd, uint8_t id, uint16_t addr,
                         uint8_t *data, uint16_t data_len, int dry_run)
{
    uint8_t params[4];
    params[0] = (uint8_t)(addr & 0xffu);
    params[1] = (uint8_t)((addr >> 8) & 0xffu);
    params[2] = (uint8_t)(data_len & 0xffu);
    params[3] = (uint8_t)((data_len >> 8) & 0xffu);

    uint8_t pkt[16];
    uint16_t pkt_len = 7u;
    pkt[0] = 0xff;
    pkt[1] = 0xff;
    pkt[2] = 0xfd;
    pkt[3] = 0x00;
    pkt[4] = id;
    pkt[5] = (uint8_t)(pkt_len & 0xffu);
    pkt[6] = (uint8_t)((pkt_len >> 8) & 0xffu);
    pkt[7] = 0x02;
    memcpy(&pkt[8], params, sizeof(params));

    uint16_t crc = 0;
    for (size_t i = 0; i < 12u; i++) {
        crc = dxl_crc_update(crc, pkt[i]);
    }
    pkt[12] = (uint8_t)(crc & 0xffu);
    pkt[13] = (uint8_t)((crc >> 8) & 0xffu);

    if (dry_run) {
        memset(data, 0, data_len);
        return 0;
    }
    tcflush(fd, TCIFLUSH);
    if (write_all(fd, pkt, 14u) != 0) {
        return -1;
    }

    uint8_t rx[64];
    size_t rx_len = 0;
    for (int tries = 0; tries < 20; tries++) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int sel = select(fd + 1, &read_fds, NULL, NULL, &tv);
        if (sel > 0 && FD_ISSET(fd, &read_fds)) {
            ssize_t n = read(fd, rx + rx_len, sizeof(rx) - rx_len);
            if (n > 0) {
                rx_len += (size_t)n;
            }
        }

        for (size_t start = 0; start + 10u <= rx_len; start++) {
            if (rx[start] != 0xff || rx[start + 1u] != 0xff ||
                rx[start + 2u] != 0xfd || rx[start + 3u] != 0x00 ||
                rx[start + 4u] != id) {
                continue;
            }
            uint16_t len = (uint16_t)rx[start + 5u] | ((uint16_t)rx[start + 6u] << 8);
            size_t total = start + 7u + len;
            if (total > rx_len) {
                continue;
            }
            if (rx[start + 7u] != 0x55 || rx[start + 8u] != 0x00 || len < (uint16_t)(data_len + 4u)) {
                return -1;
            }
            uint16_t got_crc = (uint16_t)rx[total - 2u] | ((uint16_t)rx[total - 1u] << 8);
            uint16_t calc_crc = 0;
            for (size_t i = start; i < total - 2u; i++) {
                calc_crc = dxl_crc_update(calc_crc, rx[i]);
            }
            if (got_crc != calc_crc) {
                return -1;
            }
            memcpy(data, &rx[start + 9u], data_len);
            return 0;
        }
    }
    return -1;
}

static int dxl_write1(int fd, uint8_t id, uint16_t addr, uint8_t value, int dry_run)
{
    return dxl_write_data(fd, id, addr, &value, 1u, dry_run);
}

static int dxl_write4(int fd, uint8_t id, uint16_t addr, uint32_t value, int dry_run)
{
    uint8_t data[4];
    data[0] = (uint8_t)(value & 0xffu);
    data[1] = (uint8_t)((value >> 8) & 0xffu);
    data[2] = (uint8_t)((value >> 16) & 0xffu);
    data[3] = (uint8_t)((value >> 24) & 0xffu);
    return dxl_write_data(fd, id, addr, data, 4u, dry_run);
}

static int dxl_sync_write_goal_pair(int fd, uint8_t pan_id, uint8_t tilt_id,
                                    uint32_t pan_goal, uint32_t tilt_goal,
                                    int dry_run)
{
    uint8_t pkt[32];
    const uint16_t data_len = 4u;
    const uint16_t param_len = 4u + (uint16_t)(2u * (1u + data_len));
    const uint16_t pkt_len = param_len + 3u;
    const size_t total_len = (size_t)(7u + pkt_len);
    if (total_len > sizeof(pkt)) {
        return -1;
    }

    pkt[0] = 0xff;
    pkt[1] = 0xff;
    pkt[2] = 0xfd;
    pkt[3] = 0x00;
    pkt[4] = 0xfe;
    pkt[5] = (uint8_t)(pkt_len & 0xffu);
    pkt[6] = (uint8_t)((pkt_len >> 8) & 0xffu);
    pkt[7] = 0x83;
    pkt[8] = (uint8_t)(ADDR_GOAL_POSITION & 0xffu);
    pkt[9] = (uint8_t)((ADDR_GOAL_POSITION >> 8) & 0xffu);
    pkt[10] = (uint8_t)(data_len & 0xffu);
    pkt[11] = (uint8_t)((data_len >> 8) & 0xffu);

    pkt[12] = pan_id;
    pkt[13] = (uint8_t)(pan_goal & 0xffu);
    pkt[14] = (uint8_t)((pan_goal >> 8) & 0xffu);
    pkt[15] = (uint8_t)((pan_goal >> 16) & 0xffu);
    pkt[16] = (uint8_t)((pan_goal >> 24) & 0xffu);

    pkt[17] = tilt_id;
    pkt[18] = (uint8_t)(tilt_goal & 0xffu);
    pkt[19] = (uint8_t)((tilt_goal >> 8) & 0xffu);
    pkt[20] = (uint8_t)((tilt_goal >> 16) & 0xffu);
    pkt[21] = (uint8_t)((tilt_goal >> 24) & 0xffu);

    uint16_t crc = 0;
    for (size_t i = 0; i < total_len - 2u; i++) {
        crc = dxl_crc_update(crc, pkt[i]);
    }
    pkt[total_len - 2u] = (uint8_t)(crc & 0xffu);
    pkt[total_len - 1u] = (uint8_t)((crc >> 8) & 0xffu);

    if (dry_run) {
        return 0;
    }
    return write_all(fd, pkt, total_len);
}

static int dxl_read4(int fd, uint8_t id, uint16_t addr, uint32_t *value, int dry_run)
{
    uint8_t data[4];
    if (dxl_read_data(fd, id, addr, data, 4u, dry_run) != 0) {
        return -1;
    }
    *value = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
             ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    return 0;
}

static int configure_dxl_axis(int fd, uint8_t id, uint32_t profile_accel,
                              uint32_t profile_velocity, int dry_run)
{
    if (dxl_write4(fd, id, ADDR_PROFILE_ACCEL, profile_accel, dry_run) != 0) {
        return -1;
    }
    usleep(1000);
    if (dxl_write4(fd, id, ADDR_PROFILE_VELOCITY, profile_velocity, dry_run) != 0) {
        return -1;
    }
    usleep(1000);
    if (dxl_write1(fd, id, ADDR_TORQUE_ENABLE, 1u, dry_run) != 0) {
        return -1;
    }
    return 0;
}

static int configure_dxl_pair(int fd, uint32_t pan_id, uint32_t tilt_id,
                              uint32_t profile_accel, uint32_t profile_velocity,
                              int dry_run)
{
    if (configure_dxl_axis(fd, (uint8_t)pan_id, profile_accel, profile_velocity, dry_run) != 0) {
        return -1;
    }
    usleep(1000);
    if (configure_dxl_axis(fd, (uint8_t)tilt_id, profile_accel, profile_velocity, dry_run) != 0) {
        return -1;
    }
    return 0;
}

static int send_goals_usb(int serial_fd, uint32_t pan_id, uint32_t tilt_id,
                          uint32_t pan_goal, uint32_t tilt_goal, int dry_run)
{
    if (!dry_run && serial_fd < 0) {
        return -1;
    }
    if (dxl_sync_write_goal_pair(serial_fd, (uint8_t)pan_id, (uint8_t)tilt_id,
                                 pan_goal, tilt_goal, dry_run) != 0) {
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    uint32_t base = 0;
    int udp_port = 5016;
    uint32_t pan_id = 1;
    uint32_t tilt_id = 2;
    const char *serial_path = "/dev/ttyUSB0";
    int baud = 57600;
    int dry_run = 0;
    int no_pl = 0;
    int skip_pl_init = 0;
    int skip_dxl_init = 0;
    int lazy_pl_open = 0;
    uint32_t profile_accel = 90u;
    uint32_t profile_velocity = 280u;
    const char *center_file = "/home/xilinx/ultra_yubin/front_center.env";
    uint32_t front_pan = PAN_CENTER;
    uint32_t front_tilt = TILT_CENTER;
    uint32_t sw_ctrl = 1u;
    uint32_t sw_count = 0u;
    uint32_t sw_pan = PAN_CENTER;
    uint32_t sw_tilt = TILT_CENTER;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--base") && i + 1 < argc) {
            base = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
            udp_port = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--serial") && i + 1 < argc) {
            serial_path = argv[++i];
        } else if (!strcmp(argv[i], "--baud") && i + 1 < argc) {
            baud = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--pan-id") && i + 1 < argc) {
            pan_id = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--tilt-id") && i + 1 < argc) {
            tilt_id = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--dry-run")) {
            dry_run = 1;
        } else if (!strcmp(argv[i], "--no-pl")) {
            no_pl = 1;
        } else if (!strcmp(argv[i], "--skip-pl-init")) {
            skip_pl_init = 1;
        } else if (!strcmp(argv[i], "--skip-dxl-init")) {
            skip_dxl_init = 1;
        } else if (!strcmp(argv[i], "--lazy-pl-open")) {
            lazy_pl_open = 1;
        } else if (!strcmp(argv[i], "--profile-accel") && i + 1 < argc) {
            profile_accel = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--profile-velocity") && i + 1 < argc) {
            profile_velocity = parse_u32(argv[++i]);
        } else if (!strcmp(argv[i], "--center-file") && i + 1 < argc) {
            center_file = argv[++i];
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (base == 0 && !no_pl) {
        usage(argv[0]);
        return 2;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    load_center_file(center_file, &front_pan, &front_tilt);
    sw_pan = front_pan;
    sw_tilt = front_tilt;

    int mem_fd = -1;
    void *map = MAP_FAILED;
    volatile uint32_t *regs = NULL;
    if (!no_pl && !lazy_pl_open) {
        if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
            return 1;
        }
    }

    int serial_fd = -1;
    if (!dry_run) {
        serial_fd = open_serial(serial_path, baud);
        if (serial_fd < 0) {
            perror("open serial");
            close_pl_regs(&mem_fd, &map, &regs);
            return 1;
        }
        if (!skip_dxl_init && configure_dxl_pair(serial_fd, pan_id, tilt_id,
                                                 profile_accel, profile_velocity, dry_run) != 0) {
            perror("configure dynamixel");
            close(serial_fd);
            close_pl_regs(&mem_fd, &map, &regs);
            return 1;
        }
    } else {
        (void)configure_dxl_pair(serial_fd, pan_id, tilt_id,
                                 profile_accel, profile_velocity, dry_run);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        if (serial_fd >= 0) close(serial_fd);
        close_pl_regs(&mem_fd, &map, &regs);
        return 1;
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons((uint16_t)udp_port);

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        perror("bind");
        close(sock);
        if (serial_fd >= 0) close(serial_fd);
        close_pl_regs(&mem_fd, &map, &regs);
        return 1;
    }

    if (!no_pl && !skip_pl_init) {
        if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
            close(sock);
            if (serial_fd >= 0) close(serial_fd);
            close_pl_regs(&mem_fd, &map, &regs);
            return 1;
        }
        pl_init_defaults(regs, pan_id, tilt_id, front_pan, front_tilt);
    }

    printf("[ultra_yubin] UDP listen 0.0.0.0:%d, AXI base 0x%08x\n", udp_port, base);
    printf("[ultra_yubin] serial=%s baud=%d dry_run=%d no_pl=%d skip_pl_init=%d skip_dxl_init=%d lazy_pl_open=%d pan_id=%u tilt_id=%u profile_accel=%u profile_velocity=%u center=(%u,%u) center_file=%s\n",
           serial_path, baud, dry_run, no_pl, skip_pl_init, skip_dxl_init, lazy_pl_open,
           pan_id, tilt_id, profile_accel, profile_velocity, front_pan, front_tilt, center_file);
    fflush(stdout);

    while (g_running) {
        char buf[256];
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                             (struct sockaddr *)&peer, &peer_len);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recvfrom");
            break;
        }

        buf[n] = '\0';
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r' || buf[n - 1] == ' ')) {
            buf[--n] = '\0';
        }

        if (!strcmp(buf, "PING")) {
            char reply[160];
            snprintf(reply, sizeof(reply), "PONG,UDP,ULTRA_YUBIN,base=0x%08x,port=%d,dry=%d,no_pl=%d,lazy=%d,pl_open=%d\n",
                     base, udp_port, dry_run, no_pl, lazy_pl_open, regs != NULL);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        if (!strcmp(buf, "PLPING")) {
            if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                const char *reply = "ERR,pl-open-failed\n";
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                continue;
            }
            uint32_t ctrl = no_pl ? sw_ctrl : pl_read_flags(regs);
            uint32_t count = no_pl ? sw_count : pl_read_count(regs);
            uint32_t raw_pan = no_pl ? sw_pan : pl_read_pan_goal(regs);
            uint32_t raw_tilt = no_pl ? sw_tilt : pl_read_tilt_goal(regs);
            uint32_t pan = raw_pan;
            uint32_t tilt = raw_tilt;
            const char *goal_src = no_pl ? "ps" : "pl";
            if (!goal_pair_valid(raw_pan, raw_tilt)) {
                pan = sw_pan;
                tilt = sw_tilt;
                goal_src = "ps_shadow";
            } else {
                sw_pan = raw_pan;
                sw_tilt = raw_tilt;
            }
            char reply[224];
            snprintf(reply, sizeof(reply),
                     "PONG,PL,ULTRA_YUBIN,ctrl=0x%08x,count=%u,pan=%u,tilt=%u,raw_pan=%u,raw_tilt=%u,dry=%d,no_pl=%d,src=%s\n",
                     ctrl, count, pan, tilt, raw_pan, raw_tilt, dry_run, no_pl, goal_src);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int torque_enable = 0;
        if (sscanf(buf, "TORQUE %u", &torque_enable) == 1) {
            int usb_ok = 0;
            if (torque_enable) {
                usb_ok = configure_dxl_pair(serial_fd, pan_id, tilt_id,
                                            profile_accel, profile_velocity, dry_run) == 0;
            } else {
                usb_ok = dxl_write1(serial_fd, (uint8_t)pan_id, ADDR_TORQUE_ENABLE, 0u, dry_run) == 0;
                usleep(1000);
                usb_ok = usb_ok && dxl_write1(serial_fd, (uint8_t)tilt_id, ADDR_TORQUE_ENABLE, 0u, dry_run) == 0;
            }
            char reply[96];
            snprintf(reply, sizeof(reply), "TORQUE,1,enable=%u,usb=%d,dry=%d,no_pl=%d\n",
                     torque_enable ? 1u : 0u, usb_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        if (!strcmp(buf, "READPOS")) {
            uint32_t pan_pos = sw_pan;
            uint32_t tilt_pos = sw_tilt;
            int usb_ok = 1;
            if (!dry_run) {
                usb_ok = dxl_read4(serial_fd, (uint8_t)pan_id, ADDR_PRESENT_POSITION, &pan_pos, dry_run) == 0;
                usleep(1000);
                usb_ok = usb_ok && dxl_read4(serial_fd, (uint8_t)tilt_id, ADDR_PRESENT_POSITION, &tilt_pos, dry_run) == 0;
            }
            char reply[128];
            snprintf(reply, sizeof(reply), "POS,1,pan=%u,tilt=%u,usb=%d,dry=%d,no_pl=%d\n",
                     pan_pos, tilt_pos, usb_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int set_pan = 0;
        unsigned int set_tilt = 0;
        if (!strcmp(buf, "SETCENTER") || sscanf(buf, "SETCENTER %u %u", &set_pan, &set_tilt) == 2) {
            uint32_t new_pan = set_pan;
            uint32_t new_tilt = set_tilt;
            int usb_ok = 1;
            if (!strcmp(buf, "SETCENTER")) {
                new_pan = sw_pan;
                new_tilt = sw_tilt;
                if (!dry_run) {
                    usb_ok = dxl_read4(serial_fd, (uint8_t)pan_id, ADDR_PRESENT_POSITION, &new_pan, dry_run) == 0;
                    usleep(1000);
                    usb_ok = usb_ok && dxl_read4(serial_fd, (uint8_t)tilt_id, ADDR_PRESENT_POSITION, &new_tilt, dry_run) == 0;
                }
            }
            if (new_pan <= GOAL_MAX && new_tilt <= GOAL_MAX) {
                front_pan = new_pan;
                front_tilt = new_tilt;
                sw_pan = front_pan;
                sw_tilt = front_tilt;
                if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) == 0) {
                    pl_cmd_set_goal(regs, front_pan, front_tilt);
                }
                int saved = save_center_file(center_file, front_pan, front_tilt) == 0;
                char reply[160];
                snprintf(reply, sizeof(reply), "CENTER,1,pan=%u,tilt=%u,usb=%d,saved=%d,file=%s\n",
                         front_pan, front_tilt, usb_ok, saved, center_file);
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            } else {
                const char *reply = "ERR,bad-center\n";
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            }
            continue;
        }

        if (!strcmp(buf, "CENTER")) {
            sw_pan = front_pan;
            sw_tilt = front_tilt;
            if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) == 0) {
                pl_cmd_set_goal(regs, front_pan, front_tilt);
                pl_ctrl_write(regs, 1u);
            }
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, front_pan, front_tilt, dry_run) == 0;
            char reply[128];
            snprintf(reply, sizeof(reply), "CENTER,1,pan=%u,tilt=%u,usb=%d,dry=%d,no_pl=%d\n",
                     front_pan, front_tilt, usb_ok, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int pan = 0;
        unsigned int tilt = 0;
        if (sscanf(buf, "G %u %u", &pan, &tilt) == 2) {
            if (no_pl) {
                sw_pan = pan;
                sw_tilt = tilt;
                sw_count++;
            } else {
                if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                    const char *reply = "ERR,pl-open-failed\n";
                    sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                    continue;
                }
                pl_cmd_set_goal(regs, pan, tilt);
                pl_ctrl_write(regs, 1u);
            }
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan, tilt, dry_run) == 0;
            uint32_t count = no_pl ? sw_count : pl_read_count(regs);
            char reply[160];
            snprintf(reply, sizeof(reply), "S,1,pan=%u,tilt=%u,usb=%d,count=%u,dry=%d,no_pl=%d\n",
                     pan, tilt, usb_ok, count, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        if (!strcmp(buf, "PLTEST")) {
            if (no_pl || open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                const char *reply = "PLTEST,0,reason=pl-unavailable\n";
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                continue;
            }
            uint32_t before_pan = pl_read_pan_goal(regs);
            uint32_t before_tilt = pl_read_tilt_goal(regs);
            uint32_t before_count = pl_read_count(regs);
            pl_cmd_set_goal(regs, front_pan, front_tilt);
            pl_cmd_set_track(regs, 704u, 360u, 80u, 60u, 1280u, 720u, 900u, 1u);
            usleep(1000);
            uint32_t after_pan = pl_read_pan_goal(regs);
            uint32_t after_tilt = pl_read_tilt_goal(regs);
            uint32_t after_count = pl_read_count(regs);
            uint32_t after_box = reg_read(regs, REG_TRACK_BOX);
            uint32_t after_xy = reg_read(regs, REG_TRACK_XY);
            uint32_t after_frame = reg_read(regs, REG_TRACK_FRAME);
            uint32_t after_cmd = reg_read(regs, REG_TRACK_CMD);
            char reply[320];
            snprintf(reply, sizeof(reply),
                     "PLTEST,1,before_pan=%u,before_tilt=%u,before_count=%u,after_pan=%u,after_tilt=%u,after_count=%u,box=0x%08x,xy=0x%08x,frame=0x%08x,cmd=0x%08x,front_pan=%u,front_tilt=%u\n",
                     before_pan, before_tilt, before_count, after_pan, after_tilt, after_count,
                     after_box, after_xy, after_frame, after_cmd, front_pan, front_tilt);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int cx = 0, cy = 0, bw = 0, bh = 0, fw = 0, fh = 0, conf = 0, valid = 0;
        if (sscanf(buf, "T %u %u %u %u %u %u %u %u", &cx, &cy, &bw, &bh, &fw, &fh, &conf, &valid) == 8) {
            uint32_t pan_now = sw_pan;
            uint32_t tilt_now = sw_tilt;
            uint32_t count = sw_count;
            const char *goal_src = "pl";
            if (no_pl) {
                ps_track_step(cx, cy, fw, fh, valid, front_pan, front_tilt, &sw_pan, &sw_tilt);
                pan_now = sw_pan;
                tilt_now = sw_tilt;
                sw_count++;
                count = sw_count;
                goal_src = "ps";
            } else {
                if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                    const char *reply = "ERR,pl-open-failed\n";
                    sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                    continue;
                }
                uint32_t base_pan = pl_read_pan_goal(regs);
                uint32_t base_tilt = pl_read_tilt_goal(regs);
                if (!goal_pair_near_front(base_pan, base_tilt, front_pan, front_tilt)) {
                    base_pan = sw_pan;
                    base_tilt = sw_tilt;
                }
                if (!goal_pair_near_front(base_pan, base_tilt, front_pan, front_tilt)) {
                    base_pan = front_pan;
                    base_tilt = front_tilt;
                }
                clamp_goal_pair_near_front(&base_pan, &base_tilt, front_pan, front_tilt);

                pl_cmd_set_goal(regs, base_pan, base_tilt);
                pl_cmd_set_track(regs, cx, cy, bw, bh, fw, fh, conf, valid);
                usleep(1000);
                pan_now = pl_read_pan_goal(regs);
                tilt_now = pl_read_tilt_goal(regs);
                count = pl_read_count(regs);
                if (goal_pair_valid(pan_now, tilt_now)) {
                    clamp_goal_pair_near_front(&pan_now, &tilt_now, front_pan, front_tilt);
                    pl_cmd_set_goal(regs, pan_now, tilt_now);
                    sw_pan = pan_now;
                    sw_tilt = tilt_now;
                } else {
                    ps_track_step(cx, cy, fw, fh, valid, front_pan, front_tilt, &sw_pan, &sw_tilt);
                    pan_now = sw_pan;
                    tilt_now = sw_tilt;
                    pl_cmd_set_goal(regs, pan_now, tilt_now);
                    goal_src = "ps_fallback";
                }
            }
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan_now, tilt_now, dry_run) == 0;
            char reply[224];
            snprintf(reply, sizeof(reply),
                     "T,1,cx=%u,cy=%u,bw=%u,bh=%u,fw=%u,fh=%u,conf=%u,valid=%u,pan=%u,tilt=%u,usb=%d,count=%u,dry=%d,no_pl=%d,src=%s\n",
                     cx, cy, bw, bh, fw, fh, conf, valid, pan_now, tilt_now, usb_ok, count, dry_run, no_pl, goal_src);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        int angle = 0;
        if (sscanf(buf, "A %d %u %u", &angle, &conf, &valid) == 3) {
            if (!no_pl && open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                const char *reply = "ERR,pl-open-failed\n";
                sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                continue;
            }
            uint32_t pan_now = sw_pan;
            uint32_t raw_tilt = no_pl ? sw_tilt : pl_read_tilt_goal(regs);
            uint32_t tilt_now = raw_tilt <= GOAL_MAX && raw_tilt != 0u ? raw_tilt : sw_tilt;
            uint32_t count;
            if (valid) {
                pan_now = clamp_goal_i64((int64_t)PAN_CENTER + ((int64_t)angle * AUDIO_TICKS_PER_DEG));
                sw_pan = pan_now;
                sw_tilt = tilt_now;
                sw_count++;
                if (!no_pl) {
                    pl_cmd_set_goal(regs, pan_now, tilt_now);
                }
            }
            count = sw_count;
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan_now, tilt_now, dry_run) == 0;
            char reply[160];
            snprintf(reply, sizeof(reply),
                     "A,1,angle=%d,conf=%u,valid=%u,pan=%u,tilt=%u,usb=%d,count=%u,dry=%d,no_pl=%d,ps_audio=1\n",
                     angle, conf, valid, pan_now, tilt_now, usb_ok, count, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        const char *reply = "ERR,unknown-command\n";
        sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
    }

    close(sock);
    if (serial_fd >= 0) close(serial_fd);
    close_pl_regs(&mem_fd, &map, &regs);
    return 0;
}
