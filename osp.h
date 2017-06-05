#ifndef _OSP_H
#define _OSP_H

#include <time.h>
#include <driver/driver.h>

#include "osp-transport.h"
#include "osp-protocol.h"

typedef struct osp_position {
    int32_t lat;    /* Latitude (x10^7) */
    int32_t lon;    /* Longitude (x10^7) */
    int32_t alt;    /* Altitude - above mean see level */
    uint32_t err_h; /* Horizontal error in meters*/
    uint32_t err_v; /* Vertical error in meters */
} osp_position_t;

typedef struct {
    void (*location)(void *arg, int svs, int32_t lat, int32_t lon, time_t time);
} osp_callbacks_t;

enum { OSP_INCOMING, OSP_OUTGOING };

struct osp;
typedef struct osp osp_t;

osp_t* osp_alloc(driver_t* driver, const osp_callbacks_t *cb, void *cb_arg);
int osp_start(osp_t *osp);
int osp_stop(osp_t *osp);
int osp_running(osp_t *osp);

/* OSP operations */
int osp_init(osp_t *osp, bool reset, osp_position_t *seed, uint32_t clock_drift);
int osp_factory(osp_t *osp, bool keep_prom, bool keep_xocw);
int osp_open_session(osp_t *osp, bool resume);
int osp_close_session(osp_t *osp, bool suspend);
int osp_pwr_ptf(osp_t *osp, uint32_t period, uint32_t m_search, uint32_t m_off);
int osp_pwr_full(osp_t *osp);
int osp_almanac_poll(osp_t *osp, void *almanac);
int osp_almanac_set(osp_t *osp, void *almanac);
int osp_ephemeris_poll(osp_t *osp, int svid, void *eph);
int osp_ephemeris_set(osp_t *osp, void *eph);
int osp_cw(osp_t *osp, bool enable);
int osp_set_msg_rate(osp_t *osp, uint8_t mid, uint8_t mode, uint8_t rate);

#endif /*_OSP_H */

/* vim: set ts=4 sw=4 et: */
