#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
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
        "[--skip-pl-init] [--lazy-pl-open]\n",
        argv0);
}

static uint32_t parse_u32(const char *s)
{
    return (uint32_t)strtoul(s, NULL, 0);
}

static uint32_t pack_u16(uint32_t lo, uint32_t hi)
{
    return (lo & 0xffffu) | ((hi & 0xffffu) << 16);
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

static void reg_write(volatile uint32_t *regs, uint32_t offset, uint32_t value)
{
    regs[offset / 4u] = value;
}

static uint32_t reg_read(volatile uint32_t *regs, uint32_t offset)
{
    return regs[offset / 4u];
}

static void pl_init_defaults(volatile uint32_t *regs, uint32_t pan_id, uint32_t tilt_id)
{
    reg_write(regs, REG_PAN_GOAL, 2048u);
    reg_write(regs, REG_TILT_GOAL, 2772u);
    reg_write(regs, REG_TRACK_XY, pack_u16(640u, 360u));
    reg_write(regs, REG_TRACK_FRAME, pack_u16(1280u, 720u));
    reg_write(regs, REG_IDS, ((tilt_id & 0xffu) << 8) | (pan_id & 0xffu));
    reg_write(regs, REG_CTRL, 1u);
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
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        if (crc & 1u) {
            crc = (uint16_t)((crc >> 1) ^ 0xa001u);
        } else {
            crc >>= 1;
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

static int dxl_write_goal(int fd, uint8_t id, uint32_t goal, int dry_run)
{
    uint8_t pkt[16];
    pkt[0] = 0xff;
    pkt[1] = 0xff;
    pkt[2] = 0xfd;
    pkt[3] = 0x00;
    pkt[4] = id;
    pkt[5] = 0x09;
    pkt[6] = 0x00;
    pkt[7] = 0x03;
    pkt[8] = 0x74;
    pkt[9] = 0x00;
    pkt[10] = (uint8_t)(goal & 0xffu);
    pkt[11] = (uint8_t)((goal >> 8) & 0xffu);
    pkt[12] = (uint8_t)((goal >> 16) & 0xffu);
    pkt[13] = (uint8_t)((goal >> 24) & 0xffu);

    uint16_t crc = 0;
    for (int i = 0; i < 14; i++) {
        crc = dxl_crc_update(crc, pkt[i]);
    }
    pkt[14] = (uint8_t)(crc & 0xffu);
    pkt[15] = (uint8_t)((crc >> 8) & 0xffu);

    if (dry_run) {
        return 0;
    }
    return write_all(fd, pkt, sizeof(pkt));
}

static int send_goals_usb(int serial_fd, uint32_t pan_id, uint32_t tilt_id,
                          uint32_t pan_goal, uint32_t tilt_goal, int dry_run)
{
    if (!dry_run && serial_fd < 0) {
        return -1;
    }
    if (dxl_write_goal(serial_fd, (uint8_t)pan_id, pan_goal, dry_run) != 0) {
        return -1;
    }
    usleep(1000);
    if (dxl_write_goal(serial_fd, (uint8_t)tilt_id, tilt_goal, dry_run) != 0) {
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
    int lazy_pl_open = 0;
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
        } else if (!strcmp(argv[i], "--lazy-pl-open")) {
            lazy_pl_open = 1;
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
        pl_init_defaults(regs, pan_id, tilt_id);
    }

    printf("[ultra_yubin] UDP listen 0.0.0.0:%d, AXI base 0x%08x\n", udp_port, base);
    printf("[ultra_yubin] serial=%s baud=%d dry_run=%d no_pl=%d skip_pl_init=%d lazy_pl_open=%d pan_id=%u tilt_id=%u\n",
           serial_path, baud, dry_run, no_pl, skip_pl_init, lazy_pl_open, pan_id, tilt_id);
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
            uint32_t ctrl = no_pl ? sw_ctrl : reg_read(regs, REG_CTRL);
            uint32_t count = no_pl ? sw_count : reg_read(regs, REG_STATUS);
            uint32_t pan = no_pl ? sw_pan : reg_read(regs, REG_PAN_GOAL);
            uint32_t tilt = no_pl ? sw_tilt : reg_read(regs, REG_TILT_GOAL);
            char reply[160];
            snprintf(reply, sizeof(reply), "PONG,PL,ULTRA_YUBIN,ctrl=0x%08x,count=%u,pan=%u,tilt=%u,dry=%d,no_pl=%d\n",
                     ctrl, count, pan, tilt, dry_run, no_pl);
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
                reg_write(regs, REG_PAN_GOAL, pan);
                reg_write(regs, REG_TILT_GOAL, tilt);
                reg_write(regs, REG_CTRL, 1u);
            }
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan, tilt, dry_run) == 0;
            uint32_t count = no_pl ? sw_count : reg_read(regs, REG_STATUS);
            char reply[160];
            snprintf(reply, sizeof(reply), "S,1,pan=%u,tilt=%u,usb=%d,count=%u,dry=%d,no_pl=%d\n",
                     pan, tilt, usb_ok, count, dry_run, no_pl);
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
            continue;
        }

        unsigned int cx = 0, cy = 0, bw = 0, bh = 0, fw = 0, fh = 0, conf = 0, valid = 0;
        if (sscanf(buf, "T %u %u %u %u %u %u %u %u", &cx, &cy, &bw, &bh, &fw, &fh, &conf, &valid) == 8) {
            uint32_t pan_now = sw_pan;
            uint32_t tilt_now = sw_tilt;
            uint32_t count = sw_count;
            if (no_pl) {
                sw_count++;
                count = sw_count;
            } else {
                if (open_pl_regs(base, &mem_fd, &map, &regs) != 0) {
                    const char *reply = "ERR,pl-open-failed\n";
                    sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&peer, peer_len);
                    continue;
                }
                reg_write(regs, REG_TRACK_XY, pack_u16(cx, cy));
                reg_write(regs, REG_TRACK_FRAME, pack_u16(fw, fh));
                reg_write(regs, REG_TRACK_CMD,
                          (valid ? TRACK_CMD_VALID : 0u) | TRACK_CMD_TRACK | ((conf & 0xffffu) << 8));
                usleep(1000);
                pan_now = reg_read(regs, REG_PAN_GOAL);
                tilt_now = reg_read(regs, REG_TILT_GOAL);
                count = reg_read(regs, REG_STATUS);
            }
            int usb_ok = send_goals_usb(serial_fd, pan_id, tilt_id, pan_now, tilt_now, dry_run) == 0;
            char reply[192];
            snprintf(reply, sizeof(reply),
                     "T,1,cx=%u,cy=%u,bw=%u,bh=%u,fw=%u,fh=%u,conf=%u,valid=%u,pan=%u,tilt=%u,usb=%d,count=%u,dry=%d,no_pl=%d\n",
                     cx, cy, bw, bh, fw, fh, conf, valid, pan_now, tilt_now, usb_ok, count, dry_run, no_pl);
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
            uint32_t tilt_now = no_pl ? sw_tilt : reg_read(regs, REG_TILT_GOAL);
            uint32_t count;
            if (valid) {
                pan_now = clamp_goal_i64((int64_t)PAN_CENTER + ((int64_t)angle * AUDIO_TICKS_PER_DEG));
                sw_pan = pan_now;
                sw_tilt = tilt_now;
                sw_count++;
                if (!no_pl) {
                    reg_write(regs, REG_PAN_GOAL, pan_now);
                    reg_write(regs, REG_TILT_GOAL, tilt_now);
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
