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
 //  Emlid Reach Binary (ERB) GPS driver for ROS.
 //  ERB protocol: http://files.emlid.com/ERB.pdf

#include "reach_parser.h"

GPS_ERB::GPS_ERB(GPS_State &state, const char *port = DEFAULT_PORT) :
    _step(0),
    _msg_id(0),
    _payload_length(0),
    _payload_counter(0),
    _new_position(0),
    _new_speed(0),
    _state(state),
    _tty_fd(-1),
    next_fix(NO_FIX)
{
    /* store port name */
    if(!strcmp(port,"auto"))
        strncpy(_port, GPS_ERB::find_port().c_str(), sizeof(_port));
     else
        strncpy(_port, port, sizeof(_port));
    

    /* enforce null termination */
    _port[sizeof(_port) - 1] = '\0';

    /* Check for auto port search */
    /* Initialize state object */
    GPS_ERB::_init_state();

    /* Initialize serial port */
    GPS_ERB::_init_serial();

}

GPS_ERB::~GPS_ERB()
{
    /* Close the serial port */
    close(_tty_fd);
}
// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool
GPS_ERB::read()
{
    
    /* the buffer for read chars is buflen minus null termination */
    uint8_t readbuf[sizeof(MAX_BUF_SIZE)];
    unsigned readlen = sizeof(readbuf) - 1;
    int16_t numc;
    uint8_t data;

    numc = ::read(_tty_fd, &readbuf[0], readlen);
    bool parsed = false;

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the byte
        data = readbuf[i];

        reset:
        switch(_step) {

        // Message preamble detection
        //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            Debug("reset %u", __LINE__);
            /* FALLTHROUGH */
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

        // Message header processing
        //
        case 2:
            _step++;
            _msg_id = data;
            _ck_b = _ck_a = data;                       // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length = data;                     // payload length low byte
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length += (uint16_t)(data<<8);
            _payload_counter = 0;                       // prepare to receive payload
            break;

        // Receive message data
        //
        case 5:
            _ck_b += (_ck_a += data);                   // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;

        // Checksum and message processing
        //
        case 6:
            _step++;
            if (_ck_a != data) {
                Warn("bad cka %x should be %x", data, _ck_a);
                _step = 0;
                goto reset;
            }
            break;
        case 7:
            _step = 0;
            if (_ck_b != data) {
                Warn("bad ckb %x should be %x", data, _ck_b);
                break;                                  // bad checksum
            }

            if (_parse_gps()) {
                parsed = true;
            }
            break;
        }
    }
    return parsed;
}

bool
GPS_ERB::_parse_gps(void)
{
    switch (_msg_id) {
    case MSG_VER:
        Debug("Version of ERB protocol %u.%u.%u",
              _buffer.ver.ver_high,
              _buffer.ver.ver_medium,
              _buffer.ver.ver_low);
        break;
    case MSG_POS:
        Debug("Message POS");
        _last_pos_time        = _buffer.pos.time;
        _state.longitude    = _buffer.pos.longitude;
        _state.latitude    = _buffer.pos.latitude;
        _state.altitude    = _buffer.pos.altitude_msl;
        _state.status          = next_fix;
        _new_position = true;
        _state.horizontal_accuracy = _buffer.pos.horizontal_accuracy * 1.0e-3f;// in m
        _state.vertical_accuracy = _buffer.pos.vertical_accuracy * 1.0e-3f;// in m
        _state.have_horizontal_accuracy = true;
        _state.have_vertical_accuracy = true;
        break;
    case MSG_STAT:
        Debug("Message STAT fix_status=%u fix_type=%u",
              _buffer.stat.fix_status,
              _buffer.stat.fix_type);
        if (_buffer.stat.fix_status & STAT_FIX_VALID) {
            if (_buffer.stat.fix_type == GPS_ERB::FIX_FIX) {
                next_fix = GPS_OK_FIX_3D_RTK_FIXED;
            } else if (_buffer.stat.fix_type == GPS_ERB::FIX_FLOAT) {
                next_fix = GPS_OK_FIX_3D_RTK_FLOAT;
            } else if (_buffer.stat.fix_type == GPS_ERB::FIX_SINGLE) {
                next_fix = GPS_OK_FIX_3D;
            } else {
                next_fix = NO_FIX;
                _state.status = NO_FIX;
            }
        } else {
            next_fix = NO_FIX;
            _state.status = NO_FIX;
        }
        _state.num_sats = _buffer.stat.satellites;
        if (next_fix >= GPS_OK_FIX_3D) {
            _state.time_week_ms    = _buffer.stat.time;
            _state.time_week       = _buffer.stat.week;
        }
        break;
    case MSG_DOPS:
        Debug("Message DOPS");
        _state.hdop = _buffer.dops.hDOP;
        _state.vdop = _buffer.dops.vDOP;
        break;
    case MSG_VEL:
        Debug("Message VEL");
        _last_vel_time         = _buffer.vel.time;
        _state.ground_speed     = _buffer.vel.speed_2d * 0.01f;        // m/s
        // Heading 2D deg * 100000 rescaled to deg * 100
        _state.ground_course = wr360(_buffer.vel.heading_2d * 1.0e-5f);
        _state.have_vertical_velocity = true;
        _state.velocity_x = _buffer.vel.vel_north * 0.01f;
        _state.velocity_y = _buffer.vel.vel_east * 0.01f;
        _state.velocity_z = _buffer.vel.vel_down * 0.01f;
        _state.have_speed_accuracy = true;
        _state.speed_accuracy = _buffer.vel.speed_accuracy * 0.01f;
        _new_speed = true;
        break;
    case MSG_RTK:
        Debug("Message RTK");
        _state.rtk_baseline_coords_type = RTK_BASELINE_COORDINATE_SYSTEM_NED;
        _state.rtk_num_sats      = _buffer.rtk.base_num_sats;
        if (_buffer.rtk.age_cs == 0xFFFF) {
            _state.rtk_age_ms    = 0xFFFFFFFF;
        } else {
            _state.rtk_age_ms    = _buffer.rtk.age_cs * 10;
        }
        _state.rtk_baseline_x_mm = _buffer.rtk.baseline_N_mm;
        _state.rtk_baseline_y_mm = _buffer.rtk.baseline_E_mm;
        _state.rtk_baseline_z_mm = _buffer.rtk.baseline_D_mm;
        _state.rtk_accuracy      = _buffer.rtk.ar_ratio;

        _state.rtk_week_number   = _buffer.rtk.base_week_number;
        _state.rtk_time_week_ms  = _buffer.rtk.base_time_week_ms;
        break;
    case MSG_SVI:
        Debug("Message SVI");
        /* Not processing space vehicle information */
        break;
    default:
        Warn("Unexpected message 0x%02x", (unsigned)_msg_id);
        return false;
    }
    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
        _new_speed = _new_position = false;
        _fix_count++;
        _state.fix_count = _fix_count;
        return true;
    }
    return false;
}

int32_t 
GPS_ERB::wr360(const int32_t angle, float unit_mod)
{
    const float ang_360 = 360.f * unit_mod;
    float res = fmodf(angle, ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return static_cast<int32_t>(res);
}

void 
GPS_ERB::_init_serial()
{
    /* open fd */
    _tty_fd = ::open(_port, O_RDWR | O_NOCTTY | O_SYNC);

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    unsigned speed = B115200;

    struct termios uart_config;

    int termios_state;

    tcgetattr(_tty_fd, &uart_config);

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;

    uart_config.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;         /* 8-bit characters */
    uart_config.c_cflag &= ~PARENB;     /* no parity bit */
    uart_config.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    uart_config.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    uart_config.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    uart_config.c_cc[VMIN] = 1;
    uart_config.c_cc[VTIME] = 1;

    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        Warn("ERR CFG: %d ISPD", termios_state);
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        Warn("ERR CFG: %d OSPD\n", termios_state);
    }

    if ((termios_state = tcsetattr(_tty_fd, TCSANOW, &uart_config)) < 0) {
        Warn("ERR baud %d ATTR", termios_state);
    }

    usleep(500000); // Wait for 500ms to setup 

    if (_tty_fd < 0) {
        Warn("FAIL to open");
    }
    else
    {
        Debug("Successfully opened port");
    }
}

void
GPS_ERB::_init_state()
{
    _state.status = NO_GPS;                
    _state.fix_count = 0;                 
    _state.time_week_ms = 0;              
    _state.time_week = 0;                 
    _state.latitude = 0;                     
    _state.longitude = 0;                    
    _state.altitude = 0;                    
    _state.ground_speed = 0;                 
    _state.ground_course = 0;                
    _state.hdop = 0;                      
    _state.vdop = 0;                      
    _state.num_sats = 0;                   
    _state.velocity_x = 0;                   
    _state.velocity_y = 0;                   
    _state.velocity_z = 0;                   
    _state.speed_accuracy = 0;               
    _state.horizontal_accuracy = 0;          
    _state.vertical_accuracy = 0;     
    
    _state.rtk_time_week_ms = 0;         
    _state.rtk_week_number = 0;          
    _state.rtk_age_ms = 0;               
    _state.rtk_num_sats = 0;             
    _state.rtk_baseline_coords_type = 0; 
    _state.rtk_baseline_x_mm = 0;        
    _state.rtk_baseline_y_mm = 0;        
    _state.rtk_baseline_z_mm = 0;        
    _state.rtk_accuracy = 0;             
    _state.rtk_iar_num_hypotheses = 0;
}

std::string GPS_ERB::find_port(void){
    const char* cmd = "ls /dev/serial/by-id/* | grep Edison | xargs ls -la | grep -m1 -oh '\\w*tty\\w*'";
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    if(result.length() > 10 || result.length() <= 0) // Check for valid serial port value 
        exit(-1);
    result.pop_back(); // To remove last char
    result = "/dev/" + result;
    Debug("Found Reach RTK on Port %s",result.c_str());
    return result;
}