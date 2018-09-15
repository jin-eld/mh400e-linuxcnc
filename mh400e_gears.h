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

/* Functions related to gear switching. */

#ifndef __MH400E_GEARS_H__
#define __MH400E_GEARS_H__

#include "mh400e_common.h"

/* One time setup function to prepare data structures related to gearbox 
 * switching*/
FUNCTION(gearbox_setup);

/* Construct masks from current gearbox status pins, call this function
 * once per iteration */
static void update_current_pingroup_masks(void);

/* Combine masks from each pin group to a value representing the current
 * gear setting. A return of NULL means that a corresponding value could
 * not be found, which may indicate a gearshift being in progress- */
static pair_t* get_current_gear(tree_node_t *tree);

/* Start gear shifting, parameter specifies the target gear that we want
 * to shift to.
 * ATTENTION: this function will set the vlaue of the start_gear_shift pin 
 * and also start twitching. */
static void gearshift_start(pair_t *target_gear, long period);

/* Call this function once per each thread cycle to handle gearshifting,
 * implies that gearshift_start() has been called in order to set the
 * target gear.
 *
 * Incorporates the twitching handler. */
static void gearshift_handle(long period);

/* Reset pins and state machine if an emergency stop was triggered. */
static void gearbox_handle_estop(void);

/* Returns true if a gear shifting operation is currently in progress */
static bool gearshift_in_progress(void);

/* really ugly way of keeping more order and splitting the sources,
 * halcompile does not allow to link multipe source files together, so
 * ultimately all sources need to be included by the .comp directly */
#include "mh400e_gears.c"

#endif//__MH400E_GEARS_H__
