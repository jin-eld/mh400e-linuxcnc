/*
LinuxCNC component for controlling the MAHO MH400E gearbox.

Copyright (C) 2018 Sergey 'Jin' Bostandzhyan <jin@mediatomb.cc>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#ifndef __MH400E_COMMON_H__
#define __MH400E_COMMON_H__

/* structure that allows to group pins together */
#define MH400E_PINS_IN_GROUP    4
typedef struct
{
    hal_bit_t *p[MH400E_PINS_IN_GROUP];
} pin_group_t;

/* description of a particular gear/speed setting */
typedef struct
{
    unsigned key;
    unsigned value;
} pair_t;

/* lookup table from rpm to gearbox status pin values */
static pair_t mh400e_gears[] =
{  /* rpm   bitmask                msb 11 10 9 8 7 6 5 4 3 2 1 0 lsb */
    { 0,    4       },              /* neutral           0 1 0 0 */
    { 80,   1097    },              /*   0 1 0 0 0 1 0 0 1 0 0 1 */
    { 100,  2377    },              /*   1 0 0 1 0 1 0 0 1 0 0 1 */
    { 125,  585     },              /*   1 0 1 0 0 1 0 0 1 0 0 1 */
    { 160,  1177    },              /*   0 1 0 0 1 0 0 1 1 0 0 1 */
    { 200,  2457    },              /*   1 0 0 1 1 0 0 1 1 0 0 1 */
    { 250,  665     },              /*   0 0 1 0 1 0 0 1 1 0 0 1 */
    { 315,  1065    },              /*   0 1 0 0 0 0 1 0 1 0 0 1 */
    { 400,  2345    },              /*   1 0 0 1 0 0 1 0 1 0 0 1 */
    { 500,  553     },              /*   0 0 1 0 0 0 1 0 1 0 0 1 */
    { 630,  1090    },              /*   0 1 0 0 0 1 0 0 0 0 1 0 */
    { 800,  2370    },              /*   1 0 0 1 0 1 0 0 0 0 1 0 */
    { 1000, 578     },              /*   0 0 1 0 0 1 0 0 0 0 1 0 */
    { 1250, 1170    },              /*   0 1 0 0 1 0 0 1 0 0 1 0 */
    { 1600, 2450    },              /*   1 0 0 1 1 0 0 1 0 0 1 0 */
    { 2000, 658     },              /*   0 0 1 0 1 0 0 1 0 0 1 0 */
    { 2500, 1058    },              /*   0 1 0 0 0 0 1 0 0 0 1 0 */
    { 3150, 2338    },              /*   1 0 0 1 0 0 1 0 0 0 1 0 */
    { 4000, 546     }               /*   0 0 1 0 0 0 1 0 0 0 1 0 */
};

/* total number of selectable gears including neutral */
#define MH400E_NUM_GEARS        (sizeof(mh400e_gears)/sizeof(pair_t))
/* max gear index in array */
#define MH400E_MAX_GEAR_INDEX       MH400E_NUM_GEARS - 1
/* index of neutral gear */
#define MH400E_NEUTRAL_GEAR_INDEX   0
/* min spindle rpm > 0 supported by the MH400E */
#define MH400E_MIN_RPM              80
/* index of the first non 0 rpm setting in the gears array */
#define MH400E_MIN_RPM_INDEX        1
/* max spindle rpm supported by the MH400E */
#define MH400E_MAX_RPM              4000

#endif//__MH400E_COMMON_H__