#include <stdio.h>
#include <stdlib.h>
#include <argp.h>
#include <signal.h>
#include <termios.h>
#include <syslog.h>
#include <string.h>
#include "driver/driver.h"
#include "driver/serial-io.h"
#include "osp.h"

#define execf(f) \
    if ((f)) {\
        printf(#f ": FAIL\n");\
    } else { \
        printf(#f ": SUCCESS\n");\
    }

static error_t parse_opt(int key, char *arg, struct argp_state *state);

static char doc[] = "Example of using OSP protocol";
static struct argp_option options[] = {
    {"device", 't', "DEVICE", 0,  "tty device to be used"},
    {"version", 'v', 0, 0, "print firmware version"},
    {"factory", 'f', 0, 0, "perform factory reset"},
    {"noinit", 'n', 0, 0, "do not send data initialization frame"},
    {"osp", 'o', 0, 0, "switch from NMEA to OSP protocol"},
    {"listen", 'l', 0, 0, "do not exit, listen messages"},
    { 0 }
};
static struct argp argp = { options, parse_opt, 0, doc };

struct arguments {
    char *device;
    int factory;
    int noinit;
    int osp;
    int listen;
    int version;
};

static error_t parse_opt(int key, char *arg, struct argp_state *state)
{
    struct arguments *arguments = state->input;
    switch(key)
    {
        case 'v':
            arguments->version = 1;
            break;
        case 't':
            arguments->device = arg;
            break;
        case 'f':
            arguments->factory = 1;
            break;
        case 'n':
            arguments->noinit = 1;
            break;
        case 'l':
            arguments->listen = 1;
            break;
        case 'o':
            arguments->osp = 1;
            break;
        case ARGP_KEY_ARG:
        case ARGP_KEY_END:
        default:
            return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

static void force_osp(io_t *serial, char *dev)
{
    char cmd[] = "$PSRF100,0,115200,8,1,0*04\r\n";
    int err = 0;
    serial_config(serial, dev, B4800);
    if (!err) (err = serial->open(serial));
    if (!err) (err = (serial->write(serial, cmd, sizeof(cmd)) != sizeof(cmd)));
    !err ? printf("> |%s|", cmd) : printf("error: %s\n", strerror(errno));
    usleep(100*1000); /* sending bytes */
    if (!err) (err = (serial->close(serial)));
}

static void sig_ignore(int signum)
{
}

int main(int argc, char* argv[])
{
    osp_callbacks_t cbs;
    int initialized = 0;
    struct arguments arguments;
    memset(&arguments, 0, sizeof(arguments));
    arguments.device = "/dev/ttyUSB0";

    openlog(NULL, LOG_CONS | LOG_NDELAY, LOG_USER | LOG_LOCAL0);

    signal(SIGUSR1, sig_ignore);

    argp_parse(&argp, argc, argv, 0, NULL, &arguments);

    io_t *serial = serial_alloc();
    io_t *transport = osp_transport_alloc(serial);
    driver_t *driver = driver_alloc(transport);
    osp_t *osp;
    osp = osp_alloc(driver, NULL, NULL);

    if (arguments.osp)
        force_osp(serial, arguments.device);

    serial_config(serial, arguments.device, B115200);
    osp_start(osp);
    usleep(10*1000);

    if (arguments.factory) {
        execf(osp_factory(osp, false, false));
    } else {
        if (!arguments.noinit) {
            execf(osp_init(osp, true, NULL, 0));
            execf(osp_wait_for_ready(osp));
            sleep(1); /* Wait for HW request. (FIXME) */
            execf(osp_open_session(osp, false));
        }

        if (arguments.version) {
            char version[80];
            int rv;
            execf((rv = osp_version(osp, version)));
            if (!rv) printf("Version: %s\n", version);
        }

        if (arguments.listen) {
            printf("Keep listening. Press any key to exit\n");
            getchar();
        }

        if (!arguments.noinit) {
            execf(osp_close_session(osp, false));
        }
    }

    osp_stop(osp);
    free(osp);
    free(driver);
    free(transport);
    free(serial);
    closelog();
    return 0;
}

