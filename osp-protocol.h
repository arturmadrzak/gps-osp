#ifndef _OSP_PROTOCOL_H
#define _OSP_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 1)

#define msg_begin(msg_id) struct mid##msg_id
#define msg_end __attribute__((packed))

/* Common data structures */
struct almanac_row {
    uint16_t week:10,
            status:6;
    uint16_t data[12];
    uint16_t checksum;
} msg_end;

/* Measure Navigation Data Out - MID2 (0x02) */
msg_begin(2) {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
    } position;
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } velocity;
    union {
        uint8_t byte;
        uint8_t pmode:3,
                tpmode:1,
                altmode:2,
                dop:1,
                dgps:1;
    } mode1;
    uint8_t hdop2;
    uint8_t mode2;
    uint16_t gps_week;
    uint32_t gps_tow;
    uint8_t svs_in_fix;
    uint8_t ch_prn[12];
} msg_end;

/* Measured Tracker Data Out - MID4 (0x04) */
struct mid4_ch_state {
   uint16_t acquisition:1,
            carrier_phase:1,
            bit_sync:1,
            subframe_sync:1,
            carrier_pullin:1,
            locked_code:1,
            unknown:1,
            ephemeris:1,
            reserved:8;
} ;

msg_begin(4) {
    int16_t gps_week;
    int32_t gps_tow;
    uint8_t chans;
    struct {
        uint8_t svid;
        uint8_t azimuth;
        uint8_t elev;
        uint16_t state;
        uint8_t CN0[10];
    } channel[12];
} msg_end;

/* Software version String (Response to Poll) - MID6 (0x06) */
msg_begin(6) {
    char version[80];
} msg_end;

/* Clock Status Data - MID7 (0x07) */
msg_begin(7) {
    uint16_t extended_gps_week;
    uint32_t gps_tow;
    uint8_t svs;
    uint32_t clock_drift;
    uint32_t clock_bias;
    uint32_t estimated_gps_time;
} msg_end;

/* CPU throughput - MID9 (0x09) */

/* Command Acknowledgment - MID 11 (0x0B) */
msg_begin(11) {
    uint8_t sid;
    uint8_t acid;
} msg_end;

/* Command Negative Acknowledgment - MID 12 (0x0C) */
msg_begin(12) {
    uint8_t sid;
    uint8_t nacid;
} msg_end;

/* Visible list = MID 13 (0x0D) */
msg_begin(13) {
    uint8_t svs;
    struct {
        uint8_t svid;
        int16_t azimuth;
        int16_t elevation;
    } ch[13];
} msg_end;

/* Almanac Data - MID 14 (0x0E) */
msg_begin(14) {
    uint8_t svid;
    struct almanac_row row;
} msg_end;

/* Ephemeris Data (Response to Poll) */
msg_begin(15) {
    uint8_t svid;
    uint16_t data[45];
} msg_end;

/* OkToSend - MID18 (0x12) */
enum mid18_send_indicator {
    CPU_OFF,
    CPU_ON
};

msg_begin(18) {
    uint8_t send_indicator;
} msg_end;

/* Navigation Library Measurement Data - MID28 (0x1C) */
msg_begin(28) {
    uint8_t channel;
    uint32_t time_tag;
    uint8_t svid;
    uint8_t gps_sw_time[8]; /* TODO: double 8byte */
    uint8_t pseudorange[8];
    uint8_t carrier_freq[4]; /* TODO: float*/
    uint8_t carrier_phase[8];
    uint16_t time_in_track;
    uint8_t sync_flags;
    uint8_t CN0[10];
    uint16_t delta;
    uint16_t mean_delta;
    int16_t extrapolation_time;
    uint8_t phase_error_count;
    uint8_t low_power_count;
} msg_end;

/* Navigation Library DGPS Data - MID29 (0x1D) */
msg_begin(29) {
    int16_t svid;
    int16_t iod;
    uint8_t source;
    uint8_t pseudorange_correction[4]; // float
    uint8_t pseudorange_rate_correction[4]; // float
    uint8_t correction_age[4];
    uint8_t reserved[8];
} msg_end;

/* Geodetic Navigation Data - MID41 (0x29) */
msg_begin(41) {
    union {
        uint16_t word;
    } nav_valid;
    union {
        uint16_t word;
    } nav_type;
    uint16_t extended_week_no;
    uint32_t tow;
    struct {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint16_t second;
    } utc;
    uint32_t satellite_id_list;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint8_t map_datum;
    uint16_t speed_over_ground;
    uint16_t course_over_ground;
    int16_t magnetic_variation;
    int16_t climb_rate;
    int16_t heading_rate;
    uint32_t est_h_pos_error;
    uint32_t est_v_pos_error;
    uint32_t est_time_error;
    uint16_t est_h_vel_error;
    uint32_t clock_bias;
    uint32_t clock_bias_error;
    int32_t clock_drift;
    uint32_t clock_drift_error;
    uint32_t distance;
    uint16_t distance_error;
    uint16_t heading_error;
    uint8_t svs_in_fix;
    uint8_t hdop;
    union {
        uint8_t byte;
    } add_mode_info;
} msg_end;

/* Extended Ephemeris Data/SGEE Download Output  MID56 (0x38) */
msg_begin(56) {
    uint8_t sid;
    union {
        /* Ephemeris status response*/
        struct {
            struct {
                uint8_t svid;
                uint8_t source;
                uint16_t week;
                uint16_t toe;
                uint8_t integrity;
                uint8_t age;
            } eph[12];
        } sid3;
        /* Verified 50 bps Broadcast Ephemeris and Iono Data */
        struct {
            uint8_t channel;
            uint8_t svid;
            uint32_t word;
        } sid5;
        /* SIF Status Message */
        struct {
            uint8_t sif_state;
            uint8_t cgee_state;
            uint8_t sif_aiding_type;
            uint8_t sgee_dwnld_in_progress;
            uint32_t cgee_time_left;
            uint32_t cgee_pending_mask;
            uint8_t svid_cgee_in_progress;
            uint8_t sgee_age_validity;
            uint16_t cgee_age_validity[16];
        } sid42;
    };
} msg_end;

/* Ephemeris Status Response MID70 (0x46) */
msg_begin(70) {
    uint8_t sid;
    uint8_t gps_time_flag;
    uint16_t extd_gps_week;
    uint32_t gps_tow;
    uint8_t eph_status_type;
    uint8_t gps_t_toe_limit;
    uint8_t num_svs;
    struct {
        uint8_t satid;
        uint8_t sat_info_flag;
        uint16_t gps_week;
        uint16_t gps_toe;
        uint8_t iode;
        uint16_t azimuth;
        uint8_t elevation;
    } svs[];
} msg_end;

/* Transfer Request - MID73 (0x49) */
enum mid73_transfer_type {
    TRANSFER_POSITION = 1,
    TRANSFER_TIME = 2,
    TRANSFER_FREQ = 3
};

msg_begin(73) {
    uint8_t sid; /* mid73_transfer_type */
    uint8_t multiple_request:1,
            periodic:1,
            turn_off_ref_clock:1;
} msg_end;

/* Session Opening Response - MID74 (0x4A)*/
enum mid74_status {
    SESSION_OPENING_SUCCEEDED = 0x00,
    SESSION_OPENING_FAILED = 0x01,
    SESSION_RESUME_SUCCEEDED = 0x80,
    SESSION_RESUME_FAILED = 0x81
};

msg_begin(74) {
    uint8_t sid;
    uint8_t status;
} msg_end;

/* ACK/NACK/ERROR Notification, Reject - MID75 (0x4B) */
msg_begin(75) {
    uint8_t sid;
    uint8_t echo_mid;
    uint8_t echo_sid;

    union {
        uint8_t ack;
        uint8_t reject;
    };
    uint8_t reserved[2];
} msg_end;

/* Power Mode Response - MID90 (0x5A) */
enum mid90_sid {
    FP_MODE_RESP,
    APM_RESP,
    MPM_RESP,
    ATP_RESP,
    PTF_RESP,
};

enum mid90_error_code {
    EC90_NO_ERROR,
    EC90_NO_TRANSITION,
    EC90_UNSUPPORTED,
    EC90_UNMET_PRECONDITIONS,
};

msg_begin(90) {
    uint8_t sid;
    uint8_t error_code;
} msg_end;

/* Initialize Data Source - MID128 (0x80) */
msg_begin(128) {
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    int32_t clock_drift;
    uint32_t time_of_week;
    uint16_t week_number;
    uint8_t channels;
    union {
        uint8_t byte;
        struct {
            uint8_t seed_valid:1,
                    warm:1,
                    cold:1,
                    factory:1,
                    en_nav_lib:1,
                    en_dbg:1,
                    reserved:1,
                    system_reset:1;
        } soft;
        struct {
        uint8_t keep_rom:1,
                protocol:2,
                factory:1,
                en_nav_lib:1,
                en_dbg:1,
                clr_xocw:1,
                reserved:1;
        } factory;
    };
} msg_end;

/* Set almanac - MID130 (0x82) */
msg_begin(130) {
    struct almanac_row rows[32];
} msg_end;

/* Poll software version - MID 132 (0x84) */
msg_begin(132)
{
    uint8_t reserved;
} msg_end;

/* Set protocol - MID135 (0x87) */
enum mid135_protocol {
    P_NULL,
    P_OSP,
    P_NMEA,
    P_ASCII,
    P_RTCM,
    P_USER1,
    P_SiRFLoc,
    P_STATISTIC
};

msg_begin(135) {
    uint8_t protocol;
} msg_end;

/* Poll Almanac - MID146 (0x92) */
msg_begin(146) {
    uint8_t control;
} msg_end;

/* Poll Ephemeris - MID147 (0x93)*/
msg_begin(147) {
    uint8_t svid;
    uint8_t control;
} msg_end;

/* Set Ephemeris - MID149 (0x95) */
msg_begin(149) {
    uint16_t data[45];
} msg_end;

/* Set TricklePower Parameters - MID151 (0x97) */
msg_begin(151) {
    int16_t ptf_enabled;
    int16_t duty_cycle; /* scale *10 */
    int32_t on_time;
} msg_end;

/* Set Message Rate - MID166 (0xA6) */
msg_begin(166) {
    uint8_t mode;
    uint8_t mid_to_set;
    uint8_t update_rate;
    uint8_t reserved[4];
} msg_end;

/* Ephemeries Status Request - MID212 (0xD4) */
enum mid212_sid {
    MID212_EPH_REQUEST = 1,
    MID212_ALMANAC_REQUEST = 2
};

msg_begin(212) {
    uint8_t sid;
} msg_end;

/* Session Opening/Closing Request - MID213 (0xD5) */
enum mid213_request {
    SESSION_CLOSE_REQUEST = 0x00,
    SESSION_OPEN_REQUEST = 0x71,
    SESSION_SUSPEND_REQUEST = 0x80,
    SESSION_RESUME_REQUEST = 0x80,
};

enum mid231_sid {
    SESSION_OPENING_REQUEST = 1,
    SESSION_CLOSING_REQUEST = 2,
};

msg_begin(213) {
    uint8_t sid;
    uint8_t request;
} msg_end;

/* Hardware Configuration Response - MID214 (0xD6) */
msg_begin(214) {
    struct {
        uint8_t time_ta:1,  /* Precise Time Transfer Availability */
                time_ta_dir:1,
                freq_ta:1,
                freq_ta_method:1,
                rtc_available:1,
                rtc_internal:1,
                coarse_time_ta:1,
                ref_clk:1;
    } hw_config;
    uint8_t nominal_freq[5];
    struct {
        uint8_t reserved1:2,
                aux_navmodel:1,
                navbit_aiding_123:1,
                navbit_aiding_45:1,
                reserved2:3;
    } nw_enhance_type;
} msg_end;

/* Transfer response - MID215 (0xD7) */
msg_begin(215) {
    uint8_t sid;
    union {
        struct {
            int32_t latitude;
            int32_t longitude;
            int16_t altitude;
            int8_t est_hor_err;
            int16_t est_ver_err;
            uint8_t use_alt_aiding;
        } sid1;
        struct {
            uint8_t tt_type;
            uint16_t week_number;
            uint8_t gps_time[5];
            uint8_t deltat_utc[3];
            uint8_t time_accuracy;
        } sid2;
    };
} msg_end;

/* Reject - MID216 */
msg_begin(216) {
    uint8_t sid;
    uint8_t rmid;
    uint8_t rsid;
    uint8_t reason;
} msg_end;

/* Power Mode Request - MID218 (0xDA) */
enum mid218_sid {
    PM_FULL_POWER,
    PM_APM,
    PM_MPM,
    PM_TICKLE,
    PM_PTF,
};

typedef enum {
    RTC_UNCERTAINTY_250US,
    RTC_UNCERTAINTY_125US
} mpm_rtc_opt_t;

msg_begin(218) {
    uint8_t sid;
    union {
        struct apm {
            uint8_t num_fixes;
            uint8_t tbf;
            uint8_t power_duty_cycle;
            uint8_t max_hor_err;
            uint8_t max_vert_err;
            uint8_t priority;
            uint32_t max_off_time;
            uint32_t max_search_time;
            uint8_t time_acc_priority;
        } apm;
        struct mpm {
            uint8_t timeout;
            uint8_t control;
            uint16_t reserved;
        } mpm;
        struct trickle {
            uint16_t duty_cycle;
            uint32_t on_time;
            uint32_t max_off_time;
            uint32_t max_search_time;
        } trickle;
        struct ptf {
            uint32_t period;
            uint32_t max_search_time;
            uint32_t max_off_time;
        } ptf;
    };
} msg_end;

/* CW Configuration - MID 220 (0xDC) */
typedef enum {
    CW_MODE_SCAN_AUTO,
    CW_MODE_SCAN_OFFT,
    CW_MODE_SCAN_2MHZ,
    CW_MODE_SCAN_NO_FILTER,
    CW_MODE_DISABLE,
    CW_MODE_FACTORY_SCAN = 254,
    CW_MODE_DISABLE_8F0 = 255
} cw_mode;

msg_begin(220) {
    uint8_t sid;
    uint8_t cw_mode;
} msg_end;

msg_begin(232) {
    uint8_t sid;
    uint32_t svid_mask;
} msg_end;

struct osp_frame {
    uint8_t mid;
    union {
        struct mid2 mid2;
        struct mid4 mid4;
        struct mid6 mid6;
        struct mid7 mid7;
        struct mid11 mid11;
        struct mid12 mid12;
        struct mid13 mid13;
        struct mid14 mid14;
        struct mid15 mid15;
        struct mid18 mid18;
        struct mid28 mid28;
        struct mid29 mid29;
        struct mid41 mid41;
        struct mid56 mid56;
        struct mid73 mid73;
        struct mid74 mid74;
        struct mid75 mid75;
        struct mid90 mid90;
        struct mid128 mid128;
        struct mid130 mid130;
        struct mid132 mid132;
        struct mid135 mid135;
        struct mid146 mid146;
        struct mid147 mid147;
        struct mid149 mid149;
        struct mid151 mid151;
        struct mid166 mid166;
        struct mid212 mid212;
        struct mid213 mid213;
        struct mid214 mid214;
        struct mid215 mid215;
        struct mid216 mid216;
        struct mid218 mid218;
        struct mid220 mid220;
        struct mid232 mid232;
        uint8_t reserved[256]; /* remove it */
    };
} msg_end;
typedef struct osp_frame osp_frame_t ;

#pragma pack(pop)

#endif /* _OSP_PROTOCOL_H */

/* vim: set ts=4 sw=4 et: */
