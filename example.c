#include <stdio.h>
#include <stdlib.h>
#include <argp.h>
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
    {"factory", 'f', 0, 0, "perform factory reset"},
    {"force", 'o', 0, 0, "switch from NMEA to OSP protocol"},
    {"position", 'p', "LAT,LON", 0, "seed positino"},
    { 0 }
};
static struct argp argp = { options, parse_opt, 0, doc };

struct arguments {
    char *device;
    char *eph;
    char *almanac;
    int verbose;
    int factory;
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
        case 'f':
            arguments->factory = 1;
            break;
        case 'o':
            arguments->force = 1;
            break;
        case ARGP_KEY_ARG:
        case ARGP_KEY_END:
        default:
            ARGP_ERR_UNKNOWN;
    }
    return 0;
}

static void force_osp(io_t *serial, char *dev)
{
    char cmd[] = "$PSRF100,0,115200,8,1,0*04\r\n";
    int err = 0;
    serial_config(serial, dev, 4800);
    if (!err) (err = serial->open(serial));
    if (!err) (err = (serial->write(serial, cmd, sizeof(cmd)) != sizeof(cmd)));
    usleep(100*1000); /* sending bytes */
    if (!err) (err = (serial->close(serial)));
    !err ? printf("> |%s|", cmd) : printf("error: %s\n", strerror(errno));
}

int main(int argc, char* argv[])
{
    struct arguments arguments;

    arguments.device = "/dev/ttyUSB0";
    arguments.eph = NULL;
    arguments.almanac = NULL;
    arguments.factory = 0;
    arguments.force = 0;
    arguments.verbose = 0;

    argp_parse(&argp, argc, argv, 0, NULL, &arguments);
    printf("device: %s\nverbose: %d\n", arguments.device,
            arguments.verbose);

    io_t *serial = serial_alloc();
    io_t *transport = osp_transport_alloc(serial);
    driver_t *driver = driver_alloc(transport);
    osp_t *osp = osp_alloc(driver, NULL, NULL);

    if (arguments.force)
        force_osp(serial, arguments.device);

    serial_config(serial, arguments.device, 115200);

    osp_start(osp);
    for(;;) {
        /* cmnd prompt */ 
        sleep(1);
    }

    free(osp);
    free(transport);
    free(serial);
    return 0;
}

