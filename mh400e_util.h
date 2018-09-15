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

/*
Various utilities and helper functions.
Unfortunately it is not possible to provide halcompile with multiple
sources, so we have to go the somewhat ugly way of implementing those
functions directly in the header files.
*/

#ifndef __MH400E_UTIL_H__
#define __MH400E_UTIL_H__

#include <rtapi.h>

/* Binary search tree node. */
typedef struct tree_node
{
    unsigned key;
    unsigned value;
    struct tree_node *left;
    struct tree_node *right;
} tree_node_t;

/* Allocate a tree node and assign it the given key and value.
 *
 * Note: hal_malloc() does not have a corresponding free() function,
 * this is the reason why there are no corresponding tree deallocaters.
 *
 * For our use case it's anyway not a problem, because the trees are
 * built up during intialzation and not modified anymore.
 */
static tree_node_t *tree_node_allocate(unsigned key, unsigned value);

/* Find the key of the left leaf node. */
static int tree_leaf_left(tree_node_t *node);

/* Find the key of the right leaf node. */
static int tree_leaf_right(tree_node_t *node);

/* Build up a tree from a sorted array. */
static tree_node_t *tree_from_sorted_array(pair_t *array, size_t length);

/* Return tree node by the given key.
 * If no exact match of the key was found, return the closest available.
 * This is useful when we get spindle rpm values as user in put, but
 * need to quantize them to the speeds supported by the machine. */
static tree_node_t *tree_search_closest_match(tree_node_t *root,
                                              unsigned key);

/* Return tree node by the given key, if there is no exact match (i.e.
 * key not found), return NULL. */
static tree_node_t *tree_search(tree_node_t *root, unsigned key);

/* Simple bubble sort to get our gear arrays in order. */
static void sort_array_by_key(pair_t array[], size_t length);

/* Find the closest matching gear that is supported by the MH400E.
 *
 * Everything <= 0 is matched to 0. Everything >4000 is matched to 4000,
 * which is the maximum supported speed.
 * Everything in the range 0 < rpm < 80 is matched to 80 (lowest supported
 * speed), since we assume that a value higher than zero implies spindle
 * movement.
 *
 * Note, that this function does no actually perform any pin writing operations,
 * is only quantizes the input rpm to a supported speed.
 *
 * Returns speed "pair" where rpm is stored in the "key" and the pin bitmask
 * is stored in "value".
 */
static pair_t *select_gear_from_rpm(tree_node_t *tree, float rpm);

/* really ugly way of keeping more order and splitting the sources,
 * halcompile does not allow to link multipe source files together, so
 * ultimately all sources need to be included by the .comp directly */
#include <mh400e_util.c>

#endif//__MH400E_UTIL_H__
