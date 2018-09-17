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

#include "mh400e_gears.h"
#include "mh400e_twitch.h"

typedef enum
{
    SHAFT_STATE_OFF,    /* Initial shaft state */
    SHAFT_STATE_ON,     /* Shift in process (i.e. shaft motor running) */
    SHAFT_STATE_RESTART /* Error condition, we missed our target and reached
                           an end point, we need to go back */
} shaft_state_t;

/* Group all data that is required to operate on one shaft */
typedef struct
{
    shaft_state_t state;
    pin_group_t status_pins;
    hal_bit_t *motor_on;
    hal_bit_t *motor_reverse;
    hal_bit_t *motor_slow;
    unsigned char current_mask; /* auto updated via global variable */
    unsigned char target_mask;
} shaft_data_t;

/* Group all data required for gearshifting */
static struct
{
    hal_bit_t *start_shift;
    hal_bit_t *do_stop_spindle;
    hal_bit_t *is_spindle_stopped;
    hal_bit_t *trigger_estop;
    hal_bit_t *notify_spindle_at_speed;
    bool spindle_on_before_shift;
    shaft_data_t backgear;
    shaft_data_t midrange;
    shaft_data_t input_stage;
    long delay;
    statefunc next;
} g_gearbox_data;

/* One time setup function to prepare data structures related to gearbox 
 * switching*/
FUNCTION(gearbox_setup)
{
    /* Populate data structures that will be used be the state functions
     * when shifting gears */

    g_gearbox_data.backgear.state = SHAFT_STATE_OFF;
    /* Grabbing the pin pointers in EXTRA_SETUP did not work because the
     * component did not seem to be fully initializedt there.
     *
     * Another issue:
     * while output pins are defined as (*__comp_inst->pin_name) by
     * halcompile, input pins are turned into (0+*__comp_inst->pin_name)
     * which makes it impossible to get the pointers via the defines
     * created by halcompile. Accessing the pin variable directly did not
     * work due to macro expansion, only workaround I found was to temporarily
     * disable the macros.
     */
    #pragma push_macro("reducer_left")
    #pragma push_macro("reducer_right")
    #pragma push_macro("reducer_center")
    #pragma push_macro("reducer_left_center")
    #undef reducer_left
    #undef reducer_right
    #undef reducer_center
    #undef reducer_left_center
	g_gearbox_data.backgear.status_pins = (pin_group_t)
    {
        __comp_inst->reducer_left,
        __comp_inst->reducer_right,
        __comp_inst->reducer_center,
        __comp_inst->reducer_left_center
    };
    #pragma pop_macro("reducer_left")
    #pragma pop_macro("reducer_right")
    #pragma pop_macro("reducer_center")
    #pragma pop_macro("reducer_left_center")
    g_gearbox_data.backgear.motor_on = &reducer_motor;
    g_gearbox_data.backgear.motor_reverse = &reverse_direction;
    g_gearbox_data.backgear.motor_slow = &motor_lowspeed;
    g_gearbox_data.backgear.current_mask = 0;
    g_gearbox_data.backgear.target_mask =
        mh400e_gears[MH400E_NEUTRAL_GEAR_INDEX].value; /* neutral */

    g_gearbox_data.midrange.state = SHAFT_STATE_OFF;
    #pragma push_macro("middle_left")
    #pragma push_macro("middle_right")
    #pragma push_macro("middle_center")
    #pragma push_macro("middle_left_center")
    #undef middle_left
    #undef middle_right
    #undef middle_center
    #undef middle_left_center
    g_gearbox_data.midrange.status_pins = (pin_group_t)
    {
        __comp_inst->middle_left,
        __comp_inst->middle_right,
        __comp_inst->middle_center,
        __comp_inst->middle_left_center
    };
    #pragma pop_macro("middle_left")
    #pragma pop_macro("middle_right")
    #pragma pop_macro("middle_center")
    #pragma pop_macro("middle_left_center")
    g_gearbox_data.midrange.motor_on = &midrange_motor;
    g_gearbox_data.midrange.motor_reverse = &reverse_direction;
    g_gearbox_data.midrange.motor_slow = &motor_lowspeed;
    g_gearbox_data.midrange.current_mask = 0;
    g_gearbox_data.midrange.target_mask = 0; /* don't care for neutral */

    g_gearbox_data.input_stage.state = SHAFT_STATE_OFF;
    #pragma push_macro("input_left")
    #pragma push_macro("input_right")
    #pragma push_macro("input_center")
    #pragma push_macro("input_left_center")
    #undef input_left
    #undef input_right
    #undef input_center
    #undef input_left_center
    g_gearbox_data.input_stage.status_pins = (pin_group_t)
    {
        __comp_inst->input_left,
        __comp_inst->input_right,
        __comp_inst->input_center,
        __comp_inst->input_left_center
    };
    #pragma pop_macro("input_left")
    #pragma pop_macro("input_right")
    #pragma pop_macro("input_center")
    #pragma pop_macro("input_left_center")
    g_gearbox_data.input_stage.motor_on = &input_stage_motor;
    g_gearbox_data.input_stage.motor_reverse = &reverse_direction;
    g_gearbox_data.input_stage.motor_slow = &motor_lowspeed;
    g_gearbox_data.input_stage.current_mask = 0;
    g_gearbox_data.input_stage.target_mask = 0; /* don't care for neutral */

    #pragma push_macro("spindle_stopped")
    #undef spindle_stopped
    g_gearbox_data.is_spindle_stopped = __comp_inst->spindle_stopped;
    #pragma pop_macro("spindle_stopped")
    g_gearbox_data.do_stop_spindle = &stop_spindle;
    g_gearbox_data.spindle_on_before_shift = false;
    g_gearbox_data.start_shift = &start_gear_shift;
    g_gearbox_data.trigger_estop = &estop_out;
    g_gearbox_data.notify_spindle_at_speed = &spindle_at_speed;
    g_gearbox_data.delay = 0;
    g_gearbox_data.next = NULL;
}

static void gearshift_stop_spindle(void)
{
    g_gearbox_data.spindle_on_before_shift =
        !(*g_gearbox_data.is_spindle_stopped);
    *g_gearbox_data.do_stop_spindle = true;
}

/* combine values of all pins in a group to a bitmask */
static unsigned char get_bitmask_from_pingroup(pin_group_t *group)
{
    unsigned char mask = 0;
    int i;
    for (i = 0; i < MH400E_PINS_IN_GROUP; i++)
    {
        mask |= *(group->p[i]) << i;
    }

    return mask;
}

/* Update current mask values for each shaft */
static void update_current_pingroup_masks(void)
{
    g_gearbox_data.backgear.current_mask =
        get_bitmask_from_pingroup(&g_gearbox_data.backgear.status_pins);
    g_gearbox_data.midrange.current_mask =
        get_bitmask_from_pingroup(&g_gearbox_data.midrange.status_pins);
    g_gearbox_data.input_stage.current_mask =
        get_bitmask_from_pingroup(&g_gearbox_data.input_stage.status_pins);
}

static bool estop_on_spindle_running(void)
{
    if (!*g_gearbox_data.is_spindle_stopped)
    {
        /* This is an invalid condition, spindle must be stopped if we are
         * shifting and we tested for it before we started.
         *
         * We expect that estop_out will be looped back to us so that
         * it will trigger our handler. */
        rtapi_print_msg(RTAPI_MSG_ERR, "mh400e_gearbox FATAL ERROR: detected "
                "running spindle while shifting, triggering emergency stop!\n");
        *g_gearbox_data.trigger_estop = true;
        return true;
    }

    return false;
}

/* Combine masks from each pin group to a value representing the current
 * gear setting. A return of NULL means that a corresponding value could
 * not be found, which may indicate a gearshift being in progress- */
static pair_t* get_current_gear(tree_node_t *tree)
{
    tree_node_t *result;

    unsigned combined = (g_gearbox_data.input_stage.current_mask << 8) |
                        (g_gearbox_data.midrange.current_mask << 4) |
                         g_gearbox_data.backgear.current_mask;

    /* special case: ignore all other bits for neutral */
    if (g_gearbox_data.backgear.current_mask ==
            mh400e_gears[MH400E_NEUTRAL_GEAR_INDEX].value)
    {
        return &(mh400e_gears[MH400E_NEUTRAL_GEAR_INDEX]);
    }

    result = tree_search(tree, combined);
    if (result != NULL)
    {
        return &(mh400e_gears[result->value]);
    }
    return NULL;
}

/* Helper to update delays, returns true if time has not elapsed. */
static bool gearshift_wait_delay(long period)
{
    if ((period > 0) && (g_gearbox_data.delay > 0))
    {
        g_gearbox_data.delay = g_gearbox_data.delay - period;
        return true;
    }
    g_gearbox_data.delay = 0;
    return false;
}

/* From:
 * https://forum.linuxcnc.org/12-milling/33035-retrofitting-a-1986-maho-mh400e?start=460#117021
 *
 * 1. if u need to go to the left then turn cw
 * 2. if u need to go to the right than turn ccw
 * 3. if u need to go to the middle and Left-Center is 1 then turn ccw else cw
 *
 * ┌───┐
 * ┘   └──────────────── left
 *         ┌───┐
 * ────────┘   └──────── center
 *                 ┌───┐
 * ────────────────┘   └ right
 *           ┌──────────
 * ──────────┘           left-center
 *
 */
static bool gearshift_need_reverse(unsigned char target_mask,
                                   unsigned char current_mask)
{
    if (MH400E_STAGE_IS_RIGHT(target_mask))           /* CCW, reverse is on */
    {
        return true;
    }
    else if (MH400E_STAGE_IS_LEFT(target_mask))       /* CW, reverse is off */
    {
        return false;
    }
    else if (MH400E_STAGE_IS_CENTER(target_mask))
    {
        if (!MH400E_STAGE_IS_LEFT_CENTER(current_mask))/* CW,reverse is off */
        {
            return false;
        }
    }
            
    return true;
}


/* State functions */

/* This is more or less an "overshoot" protection check in case we missed the
 * target center pos and moved further. We know when we reach an end point
 * and we know we can't continue further in this direction, so stop trying and
 * go back. Returns true if action needs to be taken. */
static bool gearshift_protect(shaft_data_t *shaft)
{
    if (!*shaft->motor_on)
    {
        return false;
    }

    if (*shaft->motor_reverse)
    {
        /* If we move to the left/CW and we reached the furthest left position
         * which does not seem to be our desired target, then we should
         * disable the motor and trigger an E-STOP, we should never end up#
         * in this situation. */
        if ((shaft->current_mask == MH400E_STAGE_POS_RIGHT) &&
            (shaft->current_mask != shaft->target_mask))
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "mh400e_gearbox: WARNING: "
                        "shaft motor at unexpected right position!\n");
        }
        else
        {
            return false;
        }
    }
    else
    {
        /* If we move to the left/CW and we reached the furthest left position
         * which does not seem to be our desired target, then we should
         * disable the motor and trigger an E-STOP, we should never end up#
         * in this situation. */
        if ((shaft->current_mask == MH400E_STAGE_POS_LEFT) &&
            (shaft->current_mask != shaft->target_mask))
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "mh400e_gearbox: WARNING: "
                        "shaft motor at unexpected left position!\n");
        }
        else
        {
            return false;
        }

    }

    return true;
}

/* Generic function that has the exact same logic, valid for all of the 
 * three shafts. */
static void gearshift_stage(shaft_data_t *shaft, statefunc me, statefunc next,
                            long period)
{
    if (estop_on_spindle_running())
    {
        return;
    }

    if (gearshift_wait_delay(period))
    {
        g_gearbox_data.next = me;
        return;
    }

    if (shaft->state == SHAFT_STATE_OFF)
    {

        /* Are the pins already in the desired state? */
        if (shaft->current_mask == shaft->target_mask)
        {
            g_gearbox_data.next = next;
        }
        else
        {
            shaft->state = SHAFT_STATE_ON;

            if (gearshift_need_reverse(shaft->target_mask,
                                       shaft->current_mask))
            {
                *shaft->motor_reverse = true;
                g_gearbox_data.delay = MH400E_REVERSE_MOTOR_INTERVAL;
            }
            g_gearbox_data.next = me;
        }
    }
    else if (shaft->state == SHAFT_STATE_ON)
    {
        /* Did we reach the desired position? */
        if (shaft->current_mask == shaft->target_mask)
        {
            if (*shaft->motor_on)
            {
                /* De-energize the shaft motor */
                *shaft->motor_on = false;
            }
            else
            {
                /* Second time we enter this state the motor will be off,
                 * that means that we already did the waiting that may have
                 * been set in the "if" below. If reverse direction was
                 * not active originally, then this does nothing */
                *shaft->motor_reverse = false;
            }

            /* If reverse direction has been set, disable it in 100ms */
            if (*shaft->motor_reverse)
            {
                g_gearbox_data.delay = MH400E_GENERIC_PIN_INTERVAL;
                g_gearbox_data.next = me;
                return;
            }
            else
            {
                *shaft->motor_slow = false;
            }
          
            if (*shaft->motor_slow)
            {
                g_gearbox_data.delay = MH400E_GENERIC_PIN_INTERVAL;
                g_gearbox_data.next = me;
                return;
            }

            /* We are done here, proceed to the next stage */
            shaft->state = SHAFT_STATE_OFF;
            g_gearbox_data.delay = MH400E_GENERIC_PIN_INTERVAL;
            g_gearbox_data.next = next;
        }
        else
        {
            /* Protect furthest lect/CW and right/CCW end positions by not
             * allowing the motor to continue running if we reached them,
             * this should never happen, but it's better to have a safety
             * measure to prevent hardware damage. The function will
             * immediately stop the motor and trigger an emergency stop if this
             * error condition is detected. */
            if (gearshift_protect(shaft))
            {
                *shaft->motor_on = false;
                shaft->state = SHAFT_STATE_RESTART;
                g_gearbox_data.delay = MH400E_REVERSE_MOTOR_INTERVAL;
                g_gearbox_data.next = me;
                return;
            }

            /* Going to the center requres lowering the motor speed */
            if (MH400E_STAGE_IS_CENTER(shaft->target_mask) && 
                !(*shaft->motor_slow))
            {
                *shaft->motor_slow = true;
            }
            else if (!(*shaft->motor_on))
            {
                /* Energize motor if it is not yet running */ 
                *shaft->motor_on = true;
            }

            g_gearbox_data.delay = MH400E_GEAR_STAGE_POLL_INTERVAL;
            g_gearbox_data.next = me;
        }
    }
    else if (shaft->state == SHAFT_STATE_RESTART)
    {
        /* Protection function restarted us, motor is already off and
         * we came here after a certain delay. We now need to check what to do
         * and re-energize */
          if (*shaft->motor_reverse)
          {
              *shaft->motor_reverse = false;
              g_gearbox_data.delay = MH400E_GENERIC_PIN_INTERVAL;
              g_gearbox_data.next = me;
              return;
          }

          if (*shaft->motor_slow)
          {
              *shaft->motor_slow = false;
              g_gearbox_data.delay = MH400E_GENERIC_PIN_INTERVAL;
          }

          /* Going back to the OFF state will retrigger the shift logic for
           * this shaft */
          shaft->state = SHAFT_STATE_OFF;
          g_gearbox_data.next = me;
    }
}

static void gearshift_stop(long period)
{
    if (gearshift_wait_delay(period))
    {
        g_gearbox_data.next = gearshift_stop;
        return;
    }

    twitch_stop(period);

    if (!twitch_stop_completed())
    {
        g_gearbox_data.delay = MH400E_TWITCH_KEEP_PIN_OFF;
        g_gearbox_data.next = gearshift_stop;
        return;
    }

    if (*g_gearbox_data.start_shift)
    {
        *g_gearbox_data.start_shift = false;

        if (g_gearbox_data.spindle_on_before_shift)
        {
            *g_gearbox_data.do_stop_spindle = false;
            g_gearbox_data.delay = MH400E_WAIT_SPINDLE_AT_SPEED;
            g_gearbox_data.next = gearshift_stop;
            return;
        }
    }

    if (g_gearbox_data.spindle_on_before_shift)
    {
        *g_gearbox_data.notify_spindle_at_speed = true;
    }

    /* We are done shifting, reset everything */
    g_gearbox_data.next = NULL;
    g_gearbox_data.spindle_on_before_shift = false;
}

static void gearshift_backgear(long period)
{
    gearshift_stage(&(g_gearbox_data.backgear), gearshift_backgear,
                    gearshift_stop, period);
}

static void gearshift_midrange(long period)
{
    gearshift_stage(&(g_gearbox_data.midrange), gearshift_midrange,
                    gearshift_backgear, period);
}

static void gearshift_input_stage(long period)
{
    gearshift_stage(&(g_gearbox_data.input_stage), gearshift_input_stage,
                    gearshift_midrange, period);
}

/* Call this function once per each thread cycle to handle gearshifting,
 * implies that gearshift_start() has been called in order to set the
 * target gear. */
static void gearshift_handle(long period)
{
    twitch_handle(period);

    if (g_gearbox_data.next == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "mh400e_gearbox FATAL ERROR: "
                        "gearshift function not set up, triggering E-Stop!\n");
        *g_gearbox_data.trigger_estop = true;
        return;
    }

    g_gearbox_data.next(period);
}

/* Start shifting process */
static void gearshift_start(pair_t *target_gear, long period)
{
    if (estop_on_spindle_running())
    {
        return;
    }

    g_gearbox_data.backgear.target_mask = (target_gear->value) & 0x000f;
    g_gearbox_data.midrange.target_mask = (target_gear->value & 0x00f0) >> 4;
    g_gearbox_data.input_stage.target_mask = 
                                    (target_gear->value & 0x0f00) >> 8;

    /* Make sure to leave 100ms between setting start_gear_shift to "on"
     * and further operations */
    g_gearbox_data.delay = MH400E_GENERIC_PIN_INTERVAL;

    *g_gearbox_data.start_shift = true;

	twitch_start(period);

    /* Special case: if we want to go to the neutral position, we
     * only care about the backgear stage, so we can jump right to it */
    if (g_gearbox_data.backgear.target_mask ==
            mh400e_gears[MH400E_NEUTRAL_GEAR_INDEX].value) {
        g_gearbox_data.next = gearshift_backgear;
    }
    else
    {
        g_gearbox_data.next = gearshift_input_stage;
    }
}

/* Reset pins and state machine if an emergency stop was triggered. */
static void gearbox_handle_estop(void)
{
    *g_gearbox_data.input_stage.motor_on = false;
    *g_gearbox_data.midrange.motor_on = false;
    *g_gearbox_data.backgear.motor_on = false;
    /* There are no separate pins for revers/slow for each shaft, each
     * shaft structure has pointers to the same pins, so its enough to
     * reset them only on one shaft. */
    *g_gearbox_data.backgear.motor_reverse = false;
    *g_gearbox_data.backgear.motor_slow = false;

    gearshift_stop(0); /* Will stop and reset twitching as well */
}

static bool gearshift_in_progress(void)
{
    return g_gearbox_data.next != NULL;
}
