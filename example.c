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
    { 0 }
};
static struct argp argp = { options, parse_opt, 0, doc };

struct arguments {
    char *device;
    char *eph;
    char *almanac;
    int verbose;
    int factory;
    int force;
    int seed;
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
    usleep(100*1000); /* sending bytes */
    if (!err) (err = (serial->close(serial)));
    !err ? printf("> |%s|", cmd) : printf("error: %s\n", strerror(errno));
}

static void poll_eph(osp_t *osp, char *filename)
{
    int count;
    ephemeris_t eph[12];
    count = osp_ephemeris_poll(osp, 0, eph);
    FILE *f = fopen(filename, "wb");
    fwrite(eph, sizeof(ephemeris_t), count, f);
    fclose(f);
}

static void poll_almanac(osp_t *osp, char *filename)
{
    almanac_t almanac;
    osp_almanac_poll(osp, &almanac);
    FILE *f = fopen(filename, "wb");
    fwrite(&almanac, sizeof(almanac_t), 1, f);
    fclose(f);
}

static int terminate = 0;

static void sig_handler(int signum)
{
    terminate = 1;
}

int main(int argc, char* argv[])
{
    struct arguments arguments;
    memset(&arguments, 0, sizeof(arguments));
    arguments.device = "/dev/ttyUSB0";

    openlog(NULL, LOG_CONS | LOG_NDELAY | LOG_PID, LOG_USER);

    signal(SIGINT, sig_handler);

    argp_parse(&argp, argc, argv, 0, NULL, &arguments);

    io_t *serial = serial_alloc();
    io_t *transport = osp_transport_alloc(serial);
    driver_t *driver = driver_alloc(transport);
    osp_t *osp = osp_alloc(driver, NULL, NULL);

    if (arguments.force)
        force_osp(serial, arguments.device);

    serial_config(serial, arguments.device, B115200);
 
    osp_start(osp);

    if (arguments.factory) {
        syslog(LOG_INFO, "osp_factory: %s\n", 
            osp_factory(osp, false, false) ? "FAIL" : "SUCCESS");
        
        usleep(100*1000);
        force_osp(serial, arguments.device);
    }

    if (arguments.seed) {
        osp_position_t seed = {
            .lat = arguments.lat,
            .lon = arguments.lon,
            .alt = arguments.alt,
            .err_h = 0,
            .err_v = 0,
        };        

        syslog(LOG_INFO, "osp_init: %s\n", 
            osp_init(osp, true, &seed, arguments.drift) ? "FAIL" : "SUCCESS");
        //set_almanac();
        //set_eph();
    }

    for(;!terminate;) {
        sleep(1);
    }

    //poll_almanac(osp, "almanac.bin");
    //poll_eph(osp, "eph.bin");

    osp_stop(osp);
    free(osp);
    free(transport);
    free(serial);
    closelog();
    return 0;
}

