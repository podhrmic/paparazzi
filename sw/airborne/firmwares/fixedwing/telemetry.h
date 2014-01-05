/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#if USE_GPS
#include "subsystems/gps.h"
#endif


#if USE_GPS
#define PERIODIC_SEND_GPS(_trans, _dev) {                                      \
    static uint8_t i;                                                   \
    int16_t climb = -gps.ned_vel.z;                                     \
    int16_t course = (DegOfRad(gps.course)/((int32_t)1e6));             \
    DOWNLINK_SEND_GPS(_trans, _dev, &gps.fix, &gps.utm_pos.east, &gps.utm_pos.north, &course, &gps.hmsl, &gps.gspeed, &climb, &gps.week, &gps.tow, &gps.utm_pos.zone, &i); \
    if ((gps.fix != GPS_FIX_3D) && (i >= gps.nb_channels)) i = 0;                                    \
    if (i >= gps.nb_channels * 2) i = 0;                                    \
    if (i < gps.nb_channels && ((gps.fix != GPS_FIX_3D) || (gps.svinfos[i].cno > 0))) { \
      DOWNLINK_SEND_SVINFO(_trans, _dev, &i, &gps.svinfos[i].svid, &gps.svinfos[i].flags, &gps.svinfos[i].qi, &gps.svinfos[i].cno, &gps.svinfos[i].elev, &gps.svinfos[i].azim); \
    }                                                                   \
    i++;                                                                \
}

#define PERIODIC_SEND_GPS_INT(_trans, _dev) {   \
  DOWNLINK_SEND_GPS_INT( _trans, _dev,          \
                         &gps.ecef_pos.x,       \
                         &gps.ecef_pos.y,       \
                         &gps.ecef_pos.z,       \
                         &gps.lla_pos.lat,      \
                         &gps.lla_pos.lon,      \
                         &gps.lla_pos.alt,      \
                         &gps.hmsl,             \
                         &gps.ecef_vel.x,       \
                         &gps.ecef_vel.y,       \
                         &gps.ecef_vel.z,       \
                         &gps.pacc,             \
                         &gps.sacc,             \
                         &gps.tow,              \
                         &gps.pdop,             \
                         &gps.num_sv,           \
                         &gps.fix);             \
  }
#else
#define PERIODIC_SEND_GPS_INT(_trans, _dev) {}
#define PERIODIC_SEND_GPS(_trans, _dev) {}
#endif

#endif /* TELEMETRY_H */
