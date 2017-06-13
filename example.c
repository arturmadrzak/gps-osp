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
        syslog(LOG_ERR, #f ": FAIL");\
    } else { \
        syslog(LOG_INFO, #f ": SUCCESS");\
    }


static error_t parse_opt(int key, char *arg, struct argp_state *state);

static char doc[] = "Example of using OSP protocol";
static struct argp_option options[] = {
    {"verbose", 'v', 0, 0, "verbose output"},
    {"device", 's', "DEVICE", 0,  "serial device to be used"},
    {"ephemeris", 'e', "FILE", 0, "ephemeris data file"},
    {"almanac", 'a', "FILE", 0, "almanac data file"},
    {"factory", 'r', 0, 0, "perform factory reset"},
    {"force", 'f', 0, 0, "switch from NMEA to OSP protocol"},
    {"position", 'p', "LAT,LON,ALT", 0, "seed position"},
    {"drift", 'd', "DRIFT", 0, "gps clock drift"},
    {"download", 'l', 0, 0, "download almanac and ephmeresis on exit"},
    {"upload", 'u', 0, 0, "upload almanac and ephmeresis on start"},
    {"listen", 't', 0, 0, "do not exit, listen messages"},
    {"measure", 'm', 0, 0, "do not exit till get fix"},
    { 0 }
};
static struct argp argp = { options, parse_opt, 0, doc };

struct arguments {
    char *device;
    char *eph;
    char *almanac;
    int verbose;
    int listen;
    int factory;
    int force;
    int seed;
    int download;
    int upload;
    int measure;
    int32_t lat, lon, alt;
    uint32_t drift;
};

static error_t parse_opt(int key, char *arg, struct argp_state *state)
{
    struct arguments *arguments = state->input;
    switch(key)
    {
        case 'v':
            arguments->verbose = 1;
            break;
        case 's':
            arguments->device = arg;
            break;
        case 'e':
            arguments->eph = arg;
            break;
        case 'a':
            arguments->almanac = arg;
            break;
        case 'r':
            arguments->factory = 1;
            break;
        case 'f':
            arguments->force = 1;
            break;
        case 'u':
            arguments->upload = 1;
            break;
        case 't':
            arguments->listen = 1;
            break;
        case 'l':
            arguments->download = 1;
            break;
        case 'm':
            arguments->measure = 1;
            break;
        case 'd':
            arguments->seed = 1;
            if (sscanf(arg, "%u", &arguments->drift) != 1)
                return ARGP_ERR_UNKNOWN;
            break;
        case 'p':
            arguments->seed = 1;
            if (sscanf(arg, "%d,%d,%d",
                &arguments->lat, &arguments->lon, &arguments->alt) != 3) {
                return ARGP_ERR_UNKNOWN;
            }
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

static void poll_eph(osp_t *osp, char *filename)
{
    int count;
    ephemeris_t eph[12];
    execf(count = osp_ephemeris_poll(osp, 0, eph) < 0);
    FILE *f = fopen(filename, "wb");
    fwrite(eph, sizeof(ephemeris_t), count, f);
    fclose(f);
}

static void poll_almanac(osp_t *osp, char *filename)
{
    almanac_t almanac;
    execf(osp_almanac_poll(osp, &almanac));
    FILE *f = fopen(filename, "wb");
    fwrite(&almanac, sizeof(almanac_t), 1, f);
    fclose(f);
}

static void set_eph(osp_t *osp, char *filename)
{
    ephemeris_t eph[12];
    int count;
    memset(eph, 0, sizeof(eph));
    FILE *f = fopen(filename, "rb");
    count = fread(eph, sizeof(ephemeris_t), 12, f);
    fclose(f);
    while(count--) {
        execf(osp_ephemeris_set(osp, eph));
    }
}

static void set_almanac(osp_t *osp, char *filename)
{
    almanac_t almanac;
    FILE *f = fopen(filename, "rb");
    fread(&almanac, sizeof(almanac_t), 1, f);
    fclose(f);
    execf(osp_almanac_set(osp, &almanac));
}

static int terminate = 0;

static void sig_handler(int signum)
{
    terminate = 1;
}

static void sig_ignore(int signum)
{
}

void location_cb(void *arg, int svs, int32_t lat, int32_t lon, time_t time)
{
    if (svs >= 3 && *(int*)arg) {
        terminate = 1;
    }
}

int main(int argc, char* argv[])
{
    osp_callbacks_t cbs;
    int initialized = 0;
    struct arguments arguments;
    memset(&arguments, 0, sizeof(arguments));
    arguments.device = "/dev/ttyUSB0";
    arguments.almanac = "almanac.bin";
    arguments.eph = "eph.bin";

    openlog(NULL, LOG_CONS | LOG_NDELAY, LOG_USER | LOG_LOCAL0);

    signal(SIGINT, sig_handler);
    signal(SIGUSR1, sig_ignore);

    argp_parse(&argp, argc, argv, 0, NULL, &arguments);

    io_t *serial = serial_alloc();
    io_t *transport = osp_transport_alloc(serial);
    driver_t *driver = driver_alloc(transport);
    osp_t *osp;

    if (arguments.measure) {
        osp = osp_alloc(driver, NULL, NULL);
    } else {
        cbs.location = location_cb;
        osp = osp_alloc(driver, &cbs, &initialized);
    }

    if (arguments.force)
        force_osp(serial, arguments.device);

    serial_config(serial, arguments.device, B115200);
 
    osp_start(osp);

    /* wait for osp running */
    usleep(10*1000);
    if (arguments.factory) {
            execf(osp_factory(osp, false, false));
    } else {
        if (arguments.seed) {
            osp_position_t seed = {
                .lat = arguments.lat,
                .lon = arguments.lon,
                .alt = arguments.alt,
                .err_h = 0,
                .err_v = 0,
            };        

            execf(osp_init(osp, true, &seed, arguments.drift));
        } else {
            execf(osp_init(osp, false, NULL, 0));
        }

        sleep(1); 

        execf(osp_open_session(osp, false));

        if (arguments.upload) {
            set_almanac(osp, arguments.almanac);
            set_eph(osp, arguments.eph);
        }

        if (arguments.download) {
            poll_almanac(osp, arguments.almanac);
            poll_eph(osp, arguments.eph);
        }
        
        if (arguments.listen || arguments.measure) {
            initialized = 1;
            for(;!terminate;)
                usleep(100*1000);
        }
    }

    execf(osp_close_session(osp, false));

    osp_stop(osp);
    free(osp);
    free(transport);
    free(serial);
    closelog();
    return 0;
}

