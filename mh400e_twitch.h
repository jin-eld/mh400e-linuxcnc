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

/* Implementation of the twitching functionality. */

#ifndef __MH400E_TWITCH_H__
#define __MH400E_TWITCH_H__

#include <rtapi.h>

#include "mh400e_common.h"

/* Call only once, sets up the global twitch state data structure */
FUNCTION(twitch_setup);

/* Call this function to start twitching.
 *
 * Makes sure that we are in a defined state (both pins are off) and
 * sets up twitch_do() */
static void twitch_start(long period);

/* Call this function once per each thread cycle to handle twitching */
static void twitch_handle(long period);

/* Call this function to stop twitching.
 *
 * Stops twitching, respecting the specified delay, always sets the
 * next function pointer to twitch_stop(). */
static void twitch_stop(long period);

/* Returns true if stop twitching operation completed. */
static bool twitch_stop_completed(void);

/* really ugly way of keeping more order and splitting the sources,
 * halcompile does not allow to link multipe source files together, so
 * ultimately all sources need to be included by the .comp directly */
#include "mh400e_twitch.c"

#endif//__MH400E_TWITCH_H__
