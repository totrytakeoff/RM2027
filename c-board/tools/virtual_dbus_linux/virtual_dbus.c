#define _GNU_SOURCE

#include <asm/termbits.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

static uint64_t now_monotonic_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int64_t clamp_i64(int64_t v, int64_t lo, int64_t hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static void die(const char *msg)
{
    fprintf(stderr, "fatal: %s (%s)\n", msg, strerror(errno));
    exit(1);
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "Usage:\n"
            "  %s --serial /dev/ttyACM0 [--kbd /dev/input/eventX] [--mouse /dev/input/eventY]\n"
            "     [--hz 70] [--mouse-scale 1.0] [--invert-y]\n"
            "     [--switch-left up|mid|down] [--switch-right up|mid|down]\n"
            "     [--dry-run] [--print-hex] [--dump /tmp/dbus.bin]\n"
            "\n"
            "Notes:\n"
            "  - Wayland 下全局键鼠采集一般需读 /dev/input/event*（evdev），需要 root 或 udev 放权。\n"
            "  - 串口使用 DT7/DR16 DBUS 参数：100000 bps, 8E1，发送 18Byte/帧。\n",
            argv0);
}

enum SwitchPos
{
    SW_UP = 1,
    SW_DOWN = 2,
    SW_MID = 3,
};

static enum SwitchPos parse_switch(const char *s)
{
    if (strcmp(s, "up") == 0)
        return SW_UP;
    if (strcmp(s, "down") == 0)
        return SW_DOWN;
    if (strcmp(s, "mid") == 0)
        return SW_MID;
    fprintf(stderr, "invalid switch pos: %s (expected up|mid|down)\n", s);
    exit(2);
}

struct RcState
{
    bool key_down[16];
    bool mouse_left;
    bool mouse_right;
    int32_t mouse_dx_acc;
    int32_t mouse_dy_acc;
};

static uint16_t keys_to_bitmask(const struct RcState *st)
{
    uint16_t mask = 0;
    for (int i = 0; i < 16; i++)
    {
        if (st->key_down[i])
            mask |= (uint16_t)(1u << i);
    }
    return mask;
}

static void build_dbus_frame(uint8_t out[18],
                             uint16_t ch0,
                             uint16_t ch1,
                             uint16_t ch2,
                             uint16_t ch3,
                             uint16_t ch4,
                             enum SwitchPos sw_left,
                             enum SwitchPos sw_right,
                             int16_t mouse_x,
                             int16_t mouse_y,
                             uint8_t mouse_l,
                             uint8_t mouse_r,
                             uint16_t keys)
{
    memset(out, 0, 18);

    ch0 &= 0x07FF;
    ch1 &= 0x07FF;
    ch2 &= 0x07FF;
    ch3 &= 0x07FF;
    ch4 &= 0x07FF;

    out[0] = (uint8_t)(ch0 & 0xFF);
    out[1] = (uint8_t)((ch0 >> 8) | ((ch1 & 0x1F) << 3));
    out[2] = (uint8_t)((ch1 >> 5) | ((ch2 & 0x03) << 6));
    out[3] = (uint8_t)((ch2 >> 2) & 0xFF);
    out[4] = (uint8_t)((ch2 >> 10) | ((ch3 & 0x7F) << 1));
    out[5] = (uint8_t)(((ch3 >> 7) & 0x0F) | ((sw_right & 0x03) << 4) | ((sw_left & 0x03) << 6));

    out[6] = (uint8_t)((uint16_t)mouse_x & 0xFF);
    out[7] = (uint8_t)(((uint16_t)mouse_x >> 8) & 0xFF);
    out[8] = (uint8_t)((uint16_t)mouse_y & 0xFF);
    out[9] = (uint8_t)(((uint16_t)mouse_y >> 8) & 0xFF);

    out[12] = mouse_l ? 1u : 0u;
    out[13] = mouse_r ? 1u : 0u;

    out[14] = (uint8_t)(keys & 0xFF);
    out[15] = (uint8_t)((keys >> 8) & 0xFF);

    out[16] = (uint8_t)(ch4 & 0xFF);
    out[17] = (uint8_t)((ch4 >> 8) & 0x07);
}

static void hex_dump_frame(uint64_t seq, const uint8_t frame[18])
{
    printf("frame #%llu:", (unsigned long long)seq);
    for (int i = 0; i < 18; ++i)
        printf(" %02X", frame[i]);
    printf("\n");
    fflush(stdout);
}

static int serial_open_100k_8e1(const char *path)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0)
        die("open serial");

    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) != 0)
        die("TCGETS2");

    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cflag = 0;

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    tio.c_cflag |= PARENB;
    tio.c_cflag &= ~PARODD;
    tio.c_cflag &= ~CSTOPB;

    tio.c_cflag &= ~CRTSCTS;

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = 100000;
    tio.c_ospeed = 100000;

    if (ioctl(fd, TCSETS2, &tio) != 0)
        die("TCSETS2");

    return fd;
}

static int open_input_device(const char *path)
{
    int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        die("open input device");
    return fd;
}

static bool test_bit(const unsigned long *bits, int bit)
{
    return (bits[bit / (int)(8 * sizeof(unsigned long))] >> (bit % (int)(8 * sizeof(unsigned long)))) & 1ul;
}

static bool device_has_key(int fd, int keycode)
{
    unsigned long keybits[(KEY_MAX + 8 * sizeof(unsigned long)) / (8 * sizeof(unsigned long)) + 1];
    memset(keybits, 0, sizeof(keybits));
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits) < 0)
        return false;
    return test_bit(keybits, keycode);
}

static bool device_has_rel(int fd, int relcode)
{
    unsigned long relbits[(REL_MAX + 8 * sizeof(unsigned long)) / (8 * sizeof(unsigned long)) + 1];
    memset(relbits, 0, sizeof(relbits));
    if (ioctl(fd, EVIOCGBIT(EV_REL, sizeof(relbits)), relbits) < 0)
        return false;
    return test_bit(relbits, relcode);
}

static bool device_is_keyboard_candidate(int fd)
{
    return device_has_key(fd, KEY_W) && device_has_key(fd, KEY_A) && device_has_key(fd, KEY_S) && device_has_key(fd, KEY_D);
}

static bool device_is_mouse_candidate(int fd)
{
    return device_has_rel(fd, REL_X) && device_has_rel(fd, REL_Y) && device_has_key(fd, BTN_LEFT) && device_has_key(fd, BTN_RIGHT);
}

static char *dup_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(NULL, 0, fmt, ap);
    va_end(ap);
    if (n < 0)
        return NULL;
    char *buf = (char *)malloc((size_t)n + 1);
    if (!buf)
        return NULL;
    va_start(ap, fmt);
    vsnprintf(buf, (size_t)n + 1, fmt, ap);
    va_end(ap);
    return buf;
}

static char *autodetect_event_device(bool want_keyboard)
{
    for (int i = 0; i < 64; i++)
    {
        char *path = dup_printf("/dev/input/event%d", i);
        if (!path)
            continue;

        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
        {
            free(path);
            continue;
        }

        bool ok = want_keyboard ? device_is_keyboard_candidate(fd) : device_is_mouse_candidate(fd);
        close(fd);
        if (ok)
            return path;
        free(path);
    }
    return NULL;
}

static void handle_input_event(struct RcState *st, const struct input_event *ev)
{
    if (ev->type == EV_KEY)
    {
        bool pressed = (ev->value != 0);
        switch (ev->code)
        {
        case KEY_W:
            st->key_down[0] = pressed;
            break;
        case KEY_S:
            st->key_down[1] = pressed;
            break;
        case KEY_D:
            st->key_down[2] = pressed;
            break;
        case KEY_A:
            st->key_down[3] = pressed;
            break;
        case KEY_LEFTSHIFT:
        case KEY_RIGHTSHIFT:
            st->key_down[4] = pressed;
            break;
        case KEY_LEFTCTRL:
        case KEY_RIGHTCTRL:
            st->key_down[5] = pressed;
            break;
        case KEY_Q:
            st->key_down[6] = pressed;
            break;
        case KEY_E:
            st->key_down[7] = pressed;
            break;
        case KEY_R:
            st->key_down[8] = pressed;
            break;
        case KEY_F:
            st->key_down[9] = pressed;
            break;
        case KEY_G:
            st->key_down[10] = pressed;
            break;
        case KEY_Z:
            st->key_down[11] = pressed;
            break;
        case KEY_X:
            st->key_down[12] = pressed;
            break;
        case KEY_C:
            st->key_down[13] = pressed;
            break;
        case KEY_V:
            st->key_down[14] = pressed;
            break;
        case KEY_B:
            st->key_down[15] = pressed;
            break;
        case BTN_LEFT:
            st->mouse_left = pressed;
            break;
        case BTN_RIGHT:
            st->mouse_right = pressed;
            break;
        default:
            break;
        }
    }
    else if (ev->type == EV_REL)
    {
        if (ev->code == REL_X)
            st->mouse_dx_acc += ev->value;
        else if (ev->code == REL_Y)
            st->mouse_dy_acc += ev->value;
    }
}

static void drain_events(int fd, struct RcState *st)
{
    struct input_event ev;
    while (1)
    {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n == (ssize_t)sizeof(ev))
        {
            handle_input_event(st, &ev);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            return;
        if (n == 0)
            return;
        if (n < 0)
            return;
        return;
    }
}

int main(int argc, char **argv)
{
    const char *serial_path = NULL;
    const char *kbd_path = NULL;
    const char *mouse_path = NULL;
    const char *dump_path = NULL;
    double mouse_scale = 1.0;
    bool invert_y = false;
    int hz = 70;
    bool dry_run = false;
    bool print_hex = false;
    enum SwitchPos sw_left = SW_UP;   // 默认键鼠模式（对应固件中 switch_is_up => MouseKeySet）
    enum SwitchPos sw_right = SW_MID; // 避免默认触发 “解除急停”

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--serial") == 0 && i + 1 < argc)
        {
            serial_path = argv[++i];
        }
        else if (strcmp(argv[i], "--kbd") == 0 && i + 1 < argc)
        {
            kbd_path = argv[++i];
        }
        else if (strcmp(argv[i], "--mouse") == 0 && i + 1 < argc)
        {
            mouse_path = argv[++i];
        }
        else if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc)
        {
            hz = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--mouse-scale") == 0 && i + 1 < argc)
        {
            mouse_scale = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--invert-y") == 0)
        {
            invert_y = true;
        }
        else if (strcmp(argv[i], "--switch-left") == 0 && i + 1 < argc)
        {
            sw_left = parse_switch(argv[++i]);
        }
        else if (strcmp(argv[i], "--switch-right") == 0 && i + 1 < argc)
        {
            sw_right = parse_switch(argv[++i]);
        }
        else if (strcmp(argv[i], "--dry-run") == 0)
        {
            dry_run = true;
        }
        else if (strcmp(argv[i], "--print-hex") == 0)
        {
            print_hex = true;
        }
        else if (strcmp(argv[i], "--dump") == 0 && i + 1 < argc)
        {
            dump_path = argv[++i];
        }
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            usage(argv[0]);
            return 0;
        }
        else
        {
            fprintf(stderr, "unknown arg: %s\n", argv[i]);
            usage(argv[0]);
            return 2;
        }
    }

    if (!serial_path && !dry_run)
    {
        usage(argv[0]);
        return 2;
    }
    if (serial_path && strcmp(serial_path, "mock") == 0)
    {
        serial_path = NULL;
        dry_run = true;
    }
    if (hz < 10)
        hz = 10;
    if (hz > 200)
        hz = 200;

    char *kbd_auto = NULL;
    char *mouse_auto = NULL;
    if (!kbd_path)
    {
        kbd_auto = autodetect_event_device(true);
        kbd_path = kbd_auto;
    }
    if (!mouse_path)
    {
        mouse_auto = autodetect_event_device(false);
        mouse_path = mouse_auto;
    }
    if (!kbd_path || !mouse_path)
    {
        fprintf(stderr, "failed to autodetect input devices (kbd=%s, mouse=%s)\n",
                kbd_path ? kbd_path : "NULL", mouse_path ? mouse_path : "NULL");
        fprintf(stderr, "hint: pass --kbd /dev/input/eventX --mouse /dev/input/eventY explicitly.\n");
        return 3;
    }

    fprintf(stderr, "[virtual_dbus] serial=%s, kbd=%s, mouse=%s, hz=%d, dry_run=%d\n",
            serial_path ? serial_path : "(none)", kbd_path, mouse_path, hz, dry_run ? 1 : 0);

    int serial_fd = -1;
    if (!dry_run && serial_path)
        serial_fd = serial_open_100k_8e1(serial_path);
    else if (!dry_run)
    {
        fprintf(stderr, "no serial device provided\n");
        return 2;
    }

    int dump_fd = -1;
    if (dump_path)
    {
        if (strcmp(dump_path, "-") == 0)
        {
            dump_fd = STDOUT_FILENO;
        }
        else
        {
            dump_fd = open(dump_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
            if (dump_fd < 0)
                die("open dump file");
        }
    }
    int kbd_fd = open_input_device(kbd_path);
    int mouse_fd = (strcmp(mouse_path, kbd_path) == 0) ? kbd_fd : open_input_device(mouse_path);

    struct RcState st;
    memset(&st, 0, sizeof(st));

    const uint64_t interval_ns = 1000000000ull / (uint64_t)hz;
    uint64_t next_send_ns = now_monotonic_ns() + interval_ns;
    uint64_t frame_seq = 0;

    while (1)
    {
        struct pollfd fds[2];
        int nfds = 0;
        fds[nfds].fd = kbd_fd;
        fds[nfds].events = POLLIN;
        nfds++;
        if (mouse_fd != kbd_fd)
        {
            fds[nfds].fd = mouse_fd;
            fds[nfds].events = POLLIN;
            nfds++;
        }

        uint64_t now_ns = now_monotonic_ns();
        int timeout_ms = 0;
        if (now_ns < next_send_ns)
        {
            uint64_t remain_ns = next_send_ns - now_ns;
            timeout_ms = (int)(remain_ns / 1000000ull);
            if (timeout_ms < 0)
                timeout_ms = 0;
            if (timeout_ms > 50)
                timeout_ms = 50;
        }

        int pr = poll(fds, nfds, timeout_ms);
        if (pr > 0)
        {
            if (fds[0].revents & POLLIN)
                drain_events(kbd_fd, &st);
            if (nfds == 2 && (fds[1].revents & POLLIN))
                drain_events(mouse_fd, &st);
        }

        now_ns = now_monotonic_ns();
        if (now_ns >= next_send_ns)
        {
            next_send_ns += interval_ns;
            if (now_ns > next_send_ns + interval_ns * 5)
                next_send_ns = now_ns + interval_ns;

            int32_t dx = st.mouse_dx_acc;
            int32_t dy = st.mouse_dy_acc;
            st.mouse_dx_acc = 0;
            st.mouse_dy_acc = 0;

            double sdx = (double)dx * mouse_scale;
            double sdy = (double)dy * mouse_scale;
            if (invert_y)
                sdy = -sdy;

            int16_t mx = (int16_t)clamp_i64((int64_t)sdx, -32768, 32767);
            int16_t my = (int16_t)clamp_i64((int64_t)sdy, -32768, 32767);

            uint8_t frame[18];
            uint16_t keys = keys_to_bitmask(&st);

            // 摇杆/拨轮默认中位，避免影响；拨轮为 0 不触发急停逻辑。
            build_dbus_frame(frame,
                             1024, 1024, 1024, 1024, 1024,
                             sw_left, sw_right,
                             mx, my,
                             st.mouse_left ? 1u : 0u,
                             st.mouse_right ? 1u : 0u,
                             keys);

            if (!dry_run && serial_fd >= 0)
            {
                ssize_t wn = write(serial_fd, frame, sizeof(frame));
                if (wn != (ssize_t)sizeof(frame))
                {
                    if (wn < 0 && errno == EINTR)
                        continue;
                    die("write serial");
                }
            }

            if (dump_fd >= 0)
                (void)write(dump_fd, frame, sizeof(frame));
            if (print_hex)
                hex_dump_frame(frame_seq, frame);

            frame_seq++;
        }
    }

    return 0;
}
