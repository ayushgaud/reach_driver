/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  Emlid Reach Binary (ERB) GPS driver for ArduPilot.
//  ERB protocol: http://files.emlid.com/ERB.pdf

#pragma once        // Only to include the header once
#include <stdint.h> // or if using C++11 then #include <cstdint> for fixed width type objetcs like int8_t etc.

#include "gps_objects.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <math.h>
#include <cstdlib>

#define DEFAULT_PORT "/dev/ttyACM0"
#define MAX_BUF_SIZE 100
#define PACKED __attribute__((__packed__))
#define FALLTHROUGH [[fallthrough]]
#define DEFINE_BYTE_ARRAY_METHODS inline uint8_t &operator[](size_t i) { return reinterpret_cast<uint8_t *>(this)[i]; }

#define ERB_DEBUGGING 1

#define STAT_FIX_VALID 0x01

#if ERB_DEBUGGING
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"
 # define Debug(fmt, args ...)  printf(GRN "%s:%d: " fmt "\n" RESET, __FUNCTION__, __LINE__, ## args)
#else
 # define Debug(fmt, args ...)
#endif

class AP_GPS_ERB
{
public:
    AP_GPS_ERB(GPS_State &state, const char *port);
    ~AP_GPS_ERB();
    // Methods
    bool read();

    static bool _detect(struct ERB_detect_state &state, uint8_t data);

private:
    
    int32_t wrap_360(const int32_t angle, float unit_mod = 1);

    struct PACKED erb_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_id;
        uint16_t length;
    };
    struct PACKED erb_ver {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        uint8_t ver_high;
        uint8_t ver_medium;
        uint8_t ver_low;
    };
    struct PACKED erb_pos {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        double longitude;
        double latitude;
        double altitude_ellipsoid;    ///< Height above ellipsoid [m]
        double altitude_msl;          ///< Height above mean sea level [m]
        uint32_t horizontal_accuracy; ///< Horizontal accuracy estimate [mm]
        uint32_t vertical_accuracy;   ///< Vertical accuracy estimate [mm]
    };
    struct PACKED erb_stat {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        uint16_t week;
        uint8_t fix_type;   ///< see erb_fix_type enum
        uint8_t fix_status;
        uint8_t satellites;
    };
    struct PACKED erb_dops {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        uint16_t gDOP;      ///< Geometric DOP
        uint16_t pDOP;      ///< Position DOP
        uint16_t vDOP;      ///< Vertical DOP
        uint16_t hDOP;      ///< Horizontal DOP
    };
    struct PACKED erb_vel {
        uint32_t time;      ///< GPS time of week of the navigation epoch [ms]
        int32_t vel_north;  ///< North velocity component [cm/s]
        int32_t vel_east;   ///< East velocity component [cm/s]
        int32_t vel_down;   ///< Down velocity component [cm/s]
        uint32_t speed_2d;  ///< Ground speed (2-D) [cm/s]
        int32_t heading_2d; ///< Heading of motion 2-D [1e5 deg]
        uint32_t speed_accuracy; ///< Speed accuracy Estimate [cm/s]
    };
    struct PACKED erb_rtk {
        uint8_t base_num_sats;       ///< Current number of satellites used for RTK calculation
        uint16_t age_cs;             ///< Age of the corrections in centiseconds (0 when no corrections, 0xFFFF indicates overflow)
        int32_t baseline_N_mm;       ///< distance between base and rover along the north axis in millimeters
        int32_t baseline_E_mm;       ///< distance between base and rover along the east axis in millimeters
        int32_t baseline_D_mm;       ///< distance between base and rover along the down axis in millimeters
        uint16_t ar_ratio;           ///< AR ratio multiplied by 10
        uint16_t base_week_number;   ///< GPS Week Number of last baseline
        uint32_t base_time_week_ms;  ///< GPS Time of Week of last baseline in milliseconds
    };

    // Receive buffer
    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        erb_ver ver;
        erb_pos pos;
        erb_stat stat;
        erb_dops dops;
        erb_vel vel;
        erb_rtk rtk;
    } _buffer;

    enum erb_protocol_bytes {
        PREAMBLE1 = 0x45,
        PREAMBLE2 = 0x52,
        MSG_VER = 0x01,
        MSG_POS = 0x02,
        MSG_STAT = 0x03,
        MSG_DOPS = 0x04,
        MSG_VEL = 0x05,
        MSG_RTK = 0x07,
    };

    enum erb_fix_type {
        FIX_NONE = 0x00,
        FIX_SINGLE = 0x01,
        FIX_FLOAT = 0x02,
        FIX_FIX = 0x03,
    };


    // Packet checksum accumulators
    uint8_t _ck_a;
    uint8_t _ck_b;

    // State machine state
    uint8_t _step;
    uint8_t _msg_id;
    uint16_t _payload_length;
    uint16_t _payload_counter;

    // 8 bit count of fix messages processed, used for periodic processing
    uint8_t _fix_count;

    uint32_t _last_pos_time;
    uint32_t _last_vel_time;

    // do we have new position information?
    bool _new_position:1;
    // do we have new speed information?
    bool _new_speed:1;

    // Buffer parse & GPS state update
    bool _parse_gps();

    // used to update fix between status and position packets
    GPS_Status next_fix;

    GPS_State &_state;

    char _port[20];

    int _tty_fd;

    void _init_serial();

    struct termios uart_config; // Serial port configuration
    
};