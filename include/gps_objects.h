/* GPS Structures to avoid circular dependencies */

#define RTK_BASELINE_COORDINATE_SYSTEM_NED 1
#define RTK_BASELINE_COORDINATE_SYSTEM_ECEF 0

struct ERB_detect_state {
    uint8_t payload_length, payload_counter;
    uint8_t step;
    uint8_t ck_a, ck_b;
};

/// GPS status codes
enum GPS_Status {
    NO_GPS,                     ///< No GPS connected/detected
    NO_FIX,                     ///< Receiving valid GPS messages but no lock
    GPS_OK_FIX_2D,              ///< Receiving valid messages and 2D lock
    GPS_OK_FIX_3D,              ///< Receiving valid messages and 3D lock
    GPS_OK_FIX_3D_DGPS,           ///< Receiving valid messages and 3D lock with differential improvements
    GPS_OK_FIX_3D_RTK_FLOAT, ///< Receiving valid messages and 3D RTK Float
    GPS_OK_FIX_3D_RTK_FIXED, ///< Receiving valid messages and 3D RTK Fixed
};

/*
  The GPS_State structure is filled in by the backend driver as it
  parses each message from the GPS.
 */
struct GPS_State {

    GPS_Status status;                  ///< driver fix status
    uint8_t fix_count;                 // 8 bit count of fix messages processed, used for periodic processing
    uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
    uint16_t time_week;                 ///< GPS week number
    float latitude;                     ///< last fix location latitude
    float longitude;                    ///< last fix location longitude
    float altitude;                    ///< last fix location longitude
    float ground_speed;                 ///< ground speed in m/sec
    float ground_course;                ///< ground course in degrees
    uint16_t hdop;                      ///< horizontal dilution of precision in cm
    uint16_t vdop;                      ///< vertical dilution of precision in cm
    uint8_t num_sats;                   ///< Number of visible satellites
    float velocity_x;                   ///< X velocity in m/s, in NED format
    float velocity_y;                   ///< Y velocity in m/s, in NED format
    float velocity_z;                   ///< Z velocity in m/s, in NED format
    float speed_accuracy;               ///< 3D velocity RMS accuracy estimate in m/s
    float horizontal_accuracy;          ///< horizontal RMS accuracy estimate in m
    float vertical_accuracy;            ///< vertical RMS accuracy estimate in m
    bool have_vertical_velocity:1;      ///< does GPS give vertical velocity? Set to true only once available.
    bool have_speed_accuracy:1;         ///< does GPS give speed accuracy? Set to true only once available.
    bool have_horizontal_accuracy:1;    ///< does GPS give horizontal position accuracy? Set to true only once available.
    bool have_vertical_accuracy:1;      ///< does GPS give vertical position accuracy? Set to true only once available.
    
    uint32_t rtk_time_week_ms;         ///< GPS Time of Week of last baseline in milliseconds
    uint16_t rtk_week_number;          ///< GPS Week Number of last baseline
    uint32_t rtk_age_ms;               ///< GPS age of last baseline correction in milliseconds  (0 when no corrections, 0xFFFFFFFF indicates overflow)
    uint8_t  rtk_num_sats;             ///< Current number of satellites used for RTK calculation
    uint8_t  rtk_baseline_coords_type; ///< Coordinate system of baseline. 0 == ECEF, 1 == NED
    int32_t  rtk_baseline_x_mm;        ///< Current baseline in ECEF x or NED north component in mm
    int32_t  rtk_baseline_y_mm;        ///< Current baseline in ECEF y or NED east component in mm
    int32_t  rtk_baseline_z_mm;        ///< Current baseline in ECEF z or NED down component in mm
    uint32_t rtk_accuracy;             ///< Current estimate of 3D baseline accuracy (receiver dependent, typical 0 to 9999)
    int32_t  rtk_iar_num_hypotheses;   ///< Current number of integer ambiguity hypotheses
};