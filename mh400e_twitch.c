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

#include "mh400e_twitch.h"

/* group twitch related data and states */
static struct
{
    bool want_cw;   /* next direction we want to twitch to */
    bool finished;  /* set by twitch_stop to signal when operation has
                       completed (twitch_stop is meant to be called repeatedly
                       in order to stop twitching while still respecting the
                       configured delays. twitch_start() */
    long delay;     /* delay in ns to do "nothing", counted down to 0 */
    hal_bit_t *cw;  /* pointer to twitch_cw pin */
    hal_bit_t *ccw; /* pointer to twitch_ccw pin */
    hal_bit_t *trigger_estop; /* set to true to trigger an emergency stop */
    statefunc next; /* next twitch state function to call */
} g_twitch_data;


/* Call only once, sets up the global twitch state data structure */
FUNCTION(twitch_setup)
{
   /* Initialize twitch data structure */
    g_twitch_data.want_cw = true;
    g_twitch_data.delay = 0;
    g_twitch_data.cw = &twitch_cw;
    g_twitch_data.ccw = &twitch_ccw;
    g_twitch_data.trigger_estop = &estop_out;
    g_twitch_data.next = twitch_stop;
    g_twitch_data.finished = true;
}

/* Call this function to stop twitching.
 *
 * Stops twitching, respecting the specified delay, always sets the
 * next function pointer to twitch_stop(). Returns "true" if stopping
 * is done (i.e. all delays have elapsed and both pins are off). */
static void twitch_stop(long period)
{
    /* Both are off - nothing to do */
    if ((*g_twitch_data.cw == false) && (*g_twitch_data.ccw == false))
    {
        g_twitch_data.delay = 0;
        g_twitch_data.next = twitch_stop;
        g_twitch_data.finished = true;
    }

    /* At least one of the pins is on, respect the delay */
    if (g_twitch_data.delay > 0)
    {
        g_twitch_data.delay = g_twitch_data.delay - period;
        g_twitch_data.next = twitch_stop;
    }

    *g_twitch_data.cw = false;
    *g_twitch_data.ccw = false;
    g_twitch_data.next = twitch_stop;
    g_twitch_data.delay = 0;
    g_twitch_data.finished = true;
}

/* Do not call this function directly, it will be setup by twitch_start().
 * Alternates between twitch_cw and twitch_ccw pins, respecting the
 * MH400E_TWITCH_KEEP_PIN_ON and MH400E_TWITCH_KEEP_PIN_OFF delays. */
static void twitch_do(long period)
{
    if (g_twitch_data.delay > 0)
    {
        g_twitch_data.delay = g_twitch_data.delay - period;
        g_twitch_data.next = twitch_do;
        return;
    }

    if ((*g_twitch_data.cw == false) && (*g_twitch_data.ccw == false))
    {
        if (g_twitch_data.want_cw)
        {
            *g_twitch_data.cw = true;
            g_twitch_data.want_cw = false;
        }
        else
        {
            *g_twitch_data.ccw = true;
            g_twitch_data.want_cw = true;
        }

        g_twitch_data.delay = MH400E_TWITCH_KEEP_PIN_ON;
        g_twitch_data.next = twitch_do;
        return;
    }
    else if (*g_twitch_data.cw == true)
    {

        *g_twitch_data.cw = false;
        g_twitch_data.want_cw = false;
        g_twitch_data.delay = MH400E_TWITCH_KEEP_PIN_OFF;
        g_twitch_data.next = twitch_do;
        return;
    }
    else if (*g_twitch_data.ccw == true)
    {
        *g_twitch_data.ccw = false;
        g_twitch_data.want_cw = true;
        g_twitch_data.delay = MH400E_TWITCH_KEEP_PIN_OFF;
        g_twitch_data.next = twitch_do;
        return;
    }
    else /* both are never allowed to be on */
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "mh400e_gearbox FATAL ERROR: twitch "
                        "cw + ccw are on, triggering emergency stop!\n");
        *g_twitch_data.trigger_estop = true;
    }
}

/* Call this function to start twitching.
 *
 * Makes sure that we are in a defined state (both pins are off) and
 * sets up twitch_do() */
static void twitch_start(long period)
{
    /* Precondition: both pins must be off before we start,
     * if they are not - stop twitching in order to get into a defined
     * state */
    if ((*g_twitch_data.cw != false) || (*g_twitch_data.ccw != false))
    {
        twitch_stop(period);
        /* stop function always resets the next pointer to twitch_stop */
        g_twitch_data.next = twitch_start;
        return;
    }

    /* Precondition is met, we can do the actual twitching now. */
    g_twitch_data.next = twitch_do;
    g_twitch_data.finished = false;
}

/* Wrapper to "hide" the global twitch_data structure */
static void twitch_handle(long period)
{
    if (g_twitch_data.next == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "mh400e_gearbox FATAL ERROR: twitch "
                        "function not set up, triggering emergency stop!\n");
        *g_twitch_data.trigger_estop = true;
        return;

    }
    g_twitch_data.next(period);
}

/* Returns true if stop twitching operation completed. */
static bool twitch_stop_completed(void)
{
    return g_twitch_data.finished;
}
