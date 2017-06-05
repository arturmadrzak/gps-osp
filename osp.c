#include "osp.h"
#include "endian.h"
//#include "geodesy.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <time.h>

#include <termios.h>

enum {
    SCAN_SKIPPED,
    SCAN_CONSUMED,
    SCAN_FINISHED,
};
typedef int (*scanner_f)(osp_t *osp, void *arg, osp_frame_t *frame, size_t len);

struct osp {
    driver_t *driver;
    osp_frame_t input;
    osp_frame_t output;

    bool busy;
    bool ready;
    pthread_mutex_t lock;
    pthread_cond_t signal;

    /* callback */
    void *arg;
    const osp_callbacks_t* callbacks;

    /* scanner */
    void *scan_arg;
    scanner_f scanner;

    /* cache */
    struct {
        struct osp_position position;
        int32_t clock_drift;
        bool valid;
    } cache;
};

#define LOG_OSP 0
#define LOG_LONG 0
#if LOG_OSP
static void log(char dir, void *frame, size_t length) {
    printf("%c (mid: %3d, length: %3d) ", dir, ((uint8_t*)frame)[0], length);
#  ifdef LOG_LONG
    int i;
    for(i = 0; i < length && i < 5; i++)
        printf("%02x", ((uint8_t*)frame)[i]);
#  endif
    printf("\n");
}
#else
#  define log(...)
#endif

static inline int osp_send(osp_t *osp, size_t length)
{
    log('>', &osp->output, length);
    return driver_send(osp->driver, &osp->output, length);
}

/* OSP internal callbacks */
static void osp_ok_to_send(osp_t *osp)
{
    osp->ready = osp->output.mid18.send_indicator;
}

static void osp_hw_config_request(osp_t *osp) 
{
    memset(&osp->output.mid214, 0, sizeof(struct mid214));
    osp->output.mid = 214;
    osp->output.mid214.hw_config.rtc_available = true;
    osp->output.mid214.hw_config.rtc_internal = true;
    osp_send(osp, 1 + sizeof(struct mid214));
}

static void osp_position_transfer_request(osp_t *osp)
{
    memset(&osp->output.mid215, 0, sizeof(struct mid215));
    osp->output.mid = 215;
    osp->output.mid215.sid = 1;

    if (osp->cache.valid) {
        printf("cache-valid: (%d, %d)\n", osp->cache.position.lat,
                osp->cache.position.lon);
        osp->output.mid215.sid1.latitude = htobe32(osp->cache.position.lat);
        osp->output.mid215.sid1.longitude =  htobe32(osp->cache.position.lon);
        osp->output.mid215.sid1.est_hor_err = 0x50; /* ~120m */
        osp->output.mid215.sid1.est_ver_err = htobe16(osp->cache.position.err_v*10);
        osp->output.mid215.sid1.use_alt_aiding = false;
        osp_send(osp, 1 + sizeof(struct mid215));
    } else {
        printf("skip. cache-invalid\n");
    }
}
static void osp_transfer_request(osp_t *osp) 
{
    uint8_t sid = osp->input.mid73.sid;
    if (sid == 1)
        osp_position_transfer_request(osp);
    else
        printf("unhandled transfer request: %d\n", sid);
}

static void osp_geodetic_nav_data(osp_t *osp)
{
    struct tm utc = {
        .tm_sec = be16toh(osp->input.mid41.utc.second)/1000,
        .tm_min = osp->input.mid41.utc.minute,
        .tm_hour = osp->input.mid41.utc.hour,
        .tm_mday = osp->input.mid41.utc.day,
        .tm_mon = osp->input.mid41.utc.month - 1,
        .tm_year = be16toh(osp->input.mid41.utc.year) - 1900,
    };
    struct mid41 *mid = &osp->input.mid41;

    if (osp->input.mid41.svs_in_fix) {
        osp->cache.clock_drift = be32toh(mid->clock_drift);
    }

    uint32_t err_h = be32toh(mid->est_h_pos_error)/100;
    uint32_t err_v = be32toh(mid->est_v_pos_error)/100;

    if (err_h < 200) {
        osp->cache.position.lat = be32toh(mid->latitude);
        osp->cache.position.lon = be32toh(mid->longitude);
        osp->cache.position.alt = be32toh(mid->altitude_msl);
        osp->cache.position.err_h = err_h;
        osp->cache.position.err_v = err_v;
    }

    printf("[%02d/%02d/%02d %02d:%02d:%02d] " \
           "nav valid: 0x%04x, nav type: 0x%04x, in fix: %d (%d, %d)\n", 
            utc.tm_year, utc.tm_mon, utc.tm_mday,
            utc.tm_hour, utc.tm_min, utc.tm_sec,
            be16toh(mid->nav_valid.word),
            be16toh(mid->nav_type.word), 
            mid->svs_in_fix,
            be32toh(mid->latitude),
            be32toh(mid->longitude)
            );

    /* mktime is broken. Substract 1 from month */
    time_t timestamp = mktime(&utc);
    if (osp->callbacks && osp->callbacks->location)
        osp->callbacks->location(osp->arg, 
                mid->svs_in_fix,
                be32toh(mid->latitude),
                be32toh(mid->longitude),
                timestamp);
}

static void osp_measure_nav_data_out(osp_t *osp)
{
#if 0
    struct mid2 *mid = &osp->input.mid2;
    osp->cache.location.x = bswap32(mid->position.x);
    osp->cache.location.y = bswap32(mid->position.y);
    osp->cache.location.z = bswap32(mid->position.z);
    printf("cache: %d, location updated (%d, %d, %d)\n",
            mid->svs_in_fix,
            osp->cache.location.x,
            osp->cache.location.y,
            osp->cache.location.z);
#endif
}

static void dump_state(struct mid4 *mid, int i)
{
    printf("0x%04x: \n", mid->channel[i].state.word);
    printf("acquisition: %d\n", mid->channel[i].state.acquisition);
    printf("carrier_phase: %d\n", mid->channel[i].state.carrier_phase);
    printf("bit_sync: %d\n", mid->channel[i].state.bit_sync);
    printf("subframe_sync: %d\n", mid->channel[i].state.subframe_sync);
    printf("carrier_pullin: %d\n", mid->channel[i].state.carrier_pullin);
    printf("locked_code: %d\n", mid->channel[i].state.locked_code);
    printf("unknown: %d\n", mid->channel[i].state.unknown);
    printf("ephemeris: %d\n", mid->channel[i].state.ephemeris);
    printf("reserved: %d\n", mid->channel[i].state.reserved);

}

static void osp_measure_tracker_data_out(osp_t *osp)
{
    struct mid4 *mid = &osp->input.mid4;
    int i, j;
    printf("CN0: ");
    for(i = 0; i < mid->chans; i++) {
        int avg = 0;
        for(j = 0; j < 10; j++)
            avg += mid->channel[i].CN0[j];
        avg /= 10;

        printf("%d(%d)0x%04x, ", mid->channel[i].svid, 
                avg,
                be16toh(mid->channel[i].state.word));
    }
    printf("\n");

    dump_state(mid, 0);
}

static void osp_clock_status_data(osp_t *osp)
{
}

static void osp_nav_lib_data(osp_t *osp)
{
}

static void osp_dispatch(osp_t *osp, osp_frame_t *frame, size_t length)
{
    if (osp->scanner) {
        int srv = osp->scanner(osp, osp->scan_arg, frame, length);
        if (srv == SCAN_FINISHED) {
            pthread_mutex_lock(&osp->lock);
            pthread_cond_signal(&osp->signal);
            pthread_mutex_unlock(&osp->lock);
        } else if (srv == SCAN_CONSUMED) {
            return;
        }
    }

    switch(frame->mid) {
        case 2:
            osp_measure_nav_data_out(osp);
            break;
        case 4:
            osp_measure_tracker_data_out(osp);
            break;
        case 7:
            osp_clock_status_data(osp);
            break;
        case 18:
            osp_ok_to_send(osp);
            break;
        case 28:
            osp_nav_lib_data(osp);
            break;
        case 41:
            osp_geodetic_nav_data(osp);
            break;
        case 73:
            osp_transfer_request(osp);
            break;
        case 9:
        case 65:
            break;
        default:
            log('<', &osp->input, length);
    }
}

static void adapter_osp_dispatch(void *arg, void* payload, size_t len) 
{
    osp_dispatch((osp_t*)arg, (osp_frame_t*)payload, len);
}

osp_t* osp_alloc(driver_t* driver, const osp_callbacks_t *cb, void *cb_arg)
{
    osp_t *osp = malloc(sizeof(osp_t));
    if (!osp) {
        errno = ENOMEM;
        return NULL;
    }
    memset(osp, 0, sizeof(osp_t));
    osp->driver = driver;
    osp->callbacks = cb;
    osp->arg = cb_arg;
    pthread_mutex_init(&osp->lock, NULL);
    pthread_cond_init(&osp->signal, NULL);
    /* configure driver */
    driver_buffer(osp->driver, &osp->input, sizeof(osp->input));
    driver_dispatcher(osp->driver, adapter_osp_dispatch, osp);
    return osp;
}

int osp_start(osp_t *osp)
{
    /* TODO: ignore gps incoming data till initialization */
    driver_enable(osp->driver);
    return 0;
}

inline int osp_running(osp_t *osp)
{
}

int osp_stop(osp_t *osp)
{
    driver_disable(osp->driver);
    return 0;
}

inline static void set_scanner(osp_t *osp, scanner_f func, void *arg)
{
    osp->scanner = func;
    osp->scan_arg = arg;
}

inline static void clr_scanner(osp_t *osp)
{
    set_scanner(osp, NULL, NULL);
}

static int transfer(osp_t *osp, size_t length, void *scanner, void *response)
{
    int retval;
    struct timespec tow; 

    if (!(retval = osp_send(osp, length)) && scanner) {
        clock_gettime(CLOCK_REALTIME, &tow); 
        tow.tv_sec += 15; 
        tow.tv_nsec = 0; 
        set_scanner(osp, scanner, response);
        retval = pthread_cond_timedwait(&osp->signal, &osp->lock, &tow);
        clr_scanner(osp);
    }     
    return retval;
}


static int init_scanner(osp_t *osp, void *arg, osp_frame_t *frame, size_t len)
{
    int rv = SCAN_SKIPPED;
    int *ack = (int*)arg;
    if (frame->mid == 11) {
        *ack = 0;
        rv = SCAN_CONSUMED;
    } else if (frame->mid == 12) {
        *ack = frame->mid12.nacid;
    } else if (frame->mid == 71) {
        osp_hw_config_request(osp);
        rv = SCAN_FINISHED;
    }
    return rv;
}

int osp_init(osp_t *osp, bool reset, osp_position_t *seed, uint32_t clock_drift)
{
    int retval = EBUSY;
    int ack = -1;

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;
        
        osp_frame_t *frame = &osp->output;
        memset(frame, 0, 1 + sizeof(struct mid128));
        frame->mid = 128;
        frame->mid128.channels = 12;
        if (seed) {
            printf("init form cache\n");
            int32_t x,y,z;
            uint32_t tow;
            uint16_t wn;
            time_t utc;
 
            /* format conversion */
            time(&utc);
            //lla_to_ecef(seed->lat, seed->lon, seed->alt,  &x, &y, &z);
            //utc_to_gps(&wn, &tow, utc);

            frame->mid128.ecef_x = htobe32(x);
            frame->mid128.ecef_y = htobe32(y);
            frame->mid128.ecef_z = htobe32(z);
            frame->mid128.clock_drift = htobe32(clock_drift);
            frame->mid128.time_of_week= htobe32(tow);
            frame->mid128.week_number = htobe16(wn);
            frame->mid128.soft.seed_valid = true;
        }

        frame->mid128.soft.system_reset = reset;

        retval = transfer(osp, 1 + sizeof(struct mid128), init_scanner, &ack);
        if (!retval && ack) {
            retval = EAGAIN;
        }
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

static int ack_scanner(osp_t *osp, void *arg, osp_frame_t *frame, size_t len)
{
    int rv = SCAN_SKIPPED;
    int *ack = (int*)arg;
    if (frame->mid == 11) {
        *ack = 0;
        rv = SCAN_FINISHED;
    }
    return rv;
}

int osp_factory(osp_t *osp, bool keep_rom, bool clr_xocw)
{
    int retval = EBUSY;
    int ack = -1;

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;
        
        osp_frame_t *frame = &osp->output;
        memset(frame, 0, 1 + sizeof(struct mid128));
        frame->mid = 128;
        frame->mid128.factory.factory = true;
        frame->mid128.factory.protocol = 3; // FIXME: return back to hardcoded OSP.
        frame->mid128.factory.clr_xocw = clr_xocw;
        frame->mid128.factory.keep_rom = keep_rom;

        retval = transfer(osp, 1 + sizeof(struct mid128), ack_scanner, &ack);
        if (!retval && ack) {
            retval = EAGAIN;
        }
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

static int session_scanner(osp_t *osp, void *arg, osp_frame_t *frame, size_t len)
{
    int retval = SCAN_SKIPPED;
    uint8_t *response = (uint8_t*)arg;
    if (frame->mid == 74) {
        response[0] = frame->mid74.sid;
        response[1] = frame->mid74.status;
        retval = SCAN_FINISHED;
    } else if (frame->mid == 75) {
        response[0] = 3;
        retval = SCAN_FINISHED;
    }
    return retval;
}

int osp_open_session(osp_t *osp, bool resume)
{
    int retval = EBUSY;
    uint8_t response[2];
    struct timespec tow;
    clock_gettime(CLOCK_REALTIME, &tow);
    tow.tv_sec += 5;
    tow.tv_nsec = 0;

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;

        osp->output.mid = 213;
        osp->output.mid213.sid = SESSION_OPENING_REQUEST;
        osp->output.mid213.request= resume ? SESSION_RESUME_REQUEST 
                                               : SESSION_OPEN_REQUEST;

        retval = transfer(osp, 1 + sizeof(struct mid213), session_scanner, response);

        if (!retval && (response[0] != 1 || response[1] != 0)) {
            retval = -1;
        }
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

int osp_close_session(osp_t *osp, bool suspend)
{
    int retval = EBUSY;
    uint8_t response[2];

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;

        osp->output.mid = 213;
        osp->output.mid213.sid = SESSION_CLOSING_REQUEST;
        osp->output.mid213.request= suspend ? SESSION_SUSPEND_REQUEST
                                               : SESSION_CLOSE_REQUEST;
        
        retval = transfer(osp, 1 + sizeof(struct mid213), session_scanner, response);
        if (!retval && (response[0] != 2 || response[1] != 0)) {
            retval = -1;
        }
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

static int pwr_ack_scanner(osp_t *osp, void *arg, osp_frame_t *frame, size_t len)
{
    int rv = SCAN_SKIPPED;
    uint8_t *data = (uint8_t*)arg;
    if (frame->mid == 90) {
        data[0] = frame->mid90.sid;
        data[1] = frame->mid90.error_code;
        rv = SCAN_FINISHED;
    }
    return rv;
}

int osp_pwr_ptf(osp_t *osp, uint32_t period, uint32_t m_search, uint32_t m_off)
{
    int retval = EBUSY;
    uint8_t response[2];

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;
 
        osp_frame_t *frame = &osp->output;
        memset(frame, 0, 1 + sizeof(struct mid218));
        frame->mid = 218;
        frame->mid218.sid = 4;
        frame->mid218.ptf.period = htobe32(period);
        frame->mid218.ptf.max_search_time = htobe32(m_search);
        frame->mid218.ptf.max_off_time = htobe32(m_off);
        
        retval = transfer(osp, 1 + 1 + sizeof(struct ptf), pwr_ack_scanner, response);
        if (!retval) {
            retval = (response[0] != 4) ? EINVAL : response[1];
        }
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

int osp_pwr_full(osp_t *osp)
{
    int retval = EBUSY;
    uint8_t response[2];
    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;
        
        osp_frame_t *frame = &osp->output;
        memset(frame, 0, 1 + sizeof(struct mid218));
        frame->mid = 218;
        frame->mid218.sid = 0;

        retval = transfer(osp, 1 + 1, pwr_ack_scanner, response);
        if (!retval && (response[0] || response[1])) {
            retval = response[1];
        }
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

static int poll_almanac_scanner(osp_t *osp, void *arg, osp_frame_t *frame, size_t len)
{
    int rv = SCAN_SKIPPED;
    uint8_t *data = (uint8_t*)arg;
    if (frame->mid == 14) {
        uint8_t svid = frame->mid14.svid - 1;
        if (svid < 32) {
            size_t size = sizeof(struct almanac_row);
            uint8_t *row = (uint8_t*)&frame->mid14.row;
            int offset = svid * size;
            memcpy(&data[offset], row, size);
            if(svid == 31) {
                rv = SCAN_FINISHED;
            } else {
                rv = SCAN_CONSUMED;
            }
        }
    }
    return rv;
}

int osp_almanac_poll(osp_t *osp, void *almanac)
{
    int retval = EBUSY;

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;

        osp_frame_t *frame = &osp->output;
        memset(frame, 0, 1 + sizeof(struct mid146));
        frame->mid = 146;
        frame->mid146.control = 0;

        retval = transfer(osp, 1 + sizeof(struct mid146), poll_almanac_scanner, almanac);

        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

int osp_almanac_set(osp_t *osp, void *almanac)
{
    int retval = EBUSY;

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;

        osp_frame_t *frame = &osp->output;
        frame->mid = 130;
        memcpy(&frame->mid130, almanac, sizeof(struct mid130));
        retval = transfer(osp, 1 + sizeof(struct mid130), NULL, NULL);
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

static int poll_eph_scanner(osp_t *osp, void *arg, osp_frame_t *frame, size_t len)
{
    int rv = SCAN_SKIPPED;
    if (frame->mid == 15) {
        uint8_t svid = frame->mid15.svid;
        printf("got eph for: %d\n", svid);
        rv = SCAN_CONSUMED;
    }
    return rv;
}

int osp_ephemeris_poll(osp_t *osp, int svid, void *eph)
{
    int retval = EBUSY;

    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;

        osp_frame_t *frame = &osp->output;
        memset(frame, 0, 1 + sizeof(struct mid147));
        frame->mid = 147;
        frame->mid147.svid = svid;

        retval = transfer(osp, 1 + sizeof(struct mid147), 
                poll_eph_scanner, eph);

        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;

}


static int cw_scanner(osp_t *osp, void *arg, osp_frame_t *frame, size_t len)
{
    int rv = SCAN_SKIPPED;
    if (frame->mid == 75) {
        printf("osp_cw: confirmed sid:%d: (%d, %d), %d\n", 
                frame->mid75.sid,
                frame->mid75.echo_mid,
                frame->mid75.echo_sid,
                frame->mid75.ack);

        rv = SCAN_FINISHED;
    }
    return rv;
}

int osp_cw(osp_t *osp, bool enable)
{
    int retval = EBUSY;
 
    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;

        osp_frame_t *frame = &osp->output;
        frame->mid = 220;
        frame->mid220.sid = 1;
        frame->mid220.cw_mode = CW_MODE_SCAN_AUTO;
        retval = transfer(osp, 1 + sizeof(struct mid220), cw_scanner, NULL);
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}


int osp_set_msg_rate(osp_t *osp, uint8_t mid, uint8_t mode, uint8_t rate)
{
    int retval = EBUSY;
 
    pthread_mutex_lock(&osp->lock);
    if (!osp->busy) {
        osp->busy = true;

        osp_frame_t *frame = &osp->output;
        frame->mid = 166;
        frame->mid166.mode = mode;
        frame->mid166.mid_to_set = mid;
        frame->mid166.update_rate = rate;
        retval = transfer(osp, 1 + sizeof(struct mid166), NULL, NULL);
        osp->busy = false;
    }
    pthread_mutex_unlock(&osp->lock);
    return retval;
}

/* vim: set ts=4 sw=4 et: */
