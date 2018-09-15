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


/* Allocate a tree node and assign it the given key and value.
 *
 * Note: hal_malloc() does not have a corresponding free() function,
 * this is the reason why there are no corresponding tree deallocaters.
 *
 * For our use case it's anyway not a problem, because the trees are
 * built up during intialzation and not modified anymore.
 */
static tree_node_t *tree_node_allocate(unsigned key, unsigned value)
{
    tree_node_t *tmp = (tree_node_t *)hal_malloc(sizeof(tree_node_t));
    if (tmp == NULL)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "MH400E_GEARBOX: failed to allocate "
                        "memory for tree node!");
        return NULL;
    }
    tmp->key = key;
    tmp->value = value;
    tmp->left = NULL;
    tmp->right = NULL;
    return tmp;
};

/* Find the key of the left leaf node. */
static int tree_leaf_left(tree_node_t *node)
{
    tree_node_t *temp = node;
    while (temp->left != NULL)
    {
        temp = temp->left;
    }
    return temp->key;
}

/* Find the key of the right leaf node. */
static int tree_leaf_right(tree_node_t *node)
{
    tree_node_t *temp = node;
    while (temp->right != NULL)
    {
        temp = temp->right;
    }
    return temp->key;
}

/* Build up a tree from a sorted array. */
static tree_node_t *tree_from_sorted_array(pair_t *array, size_t length)
{
    int i;
    unsigned p = 1;

    /* find the smallest power of two larger than length */
    while (p < length)
    {
        p = p << 1;
    }

    tree_node_t *ptr[p];
    for (i = 0; i < p; i++)
    {
        ptr[i] = NULL;
    }

    /* scale array to last "ideal" tree level */
    for (i = 0; i < length; i++)
    {
        unsigned j = i * p / length;
        ptr[j] = tree_node_allocate(array[i].key, array[i].value);
    }

    while (p > 1)
    {
        for (i = 0; i < p; i += 2)
        {
            /* three cases */
            if (ptr[i] && ptr[i + 1]) /* (leaf, leaf) */
            {
                tree_node_t *dn = tree_node_allocate(0, 0);
                if (dn == NULL)
                {
                    /* Fatal: allocation failed, error message will
                     * be printed by the allocate function. */
                    return NULL;
                }

                dn->left = ptr[i];
                dn->right = ptr[i + 1];
                dn->key = ((tree_leaf_left(dn->right) +
                            tree_leaf_right(dn->left)) / 2);
                ptr[i / 2] = dn;
            }
            else if (ptr[i]) /* (leaf, dummy) */
            {
                ptr[i / 2] = ptr[i];
            } else if (ptr[i+1]) /* (dummy, leaf) */
            {
                ptr[i / 2] = ptr[i + 1];
            } /* (dummy, dummy) not possible due to scaling */
        }
        p = p >> 1;
    }
    /* return root node */
    return ptr[0];
}

/* Return tree node by the given key.
 * If no exact match of the key was found, return the closest available.
 * This is useful when we get spindle rpm values as user in put, but
 * need to quantize them to the speeds supported by the machine. */
static tree_node_t *tree_search_closest_match(tree_node_t *root, unsigned key)
{
    if (root == NULL)
    {
        return NULL;
    }

    /* our values are only stored in leafs */
    if ((root->left == NULL) && (root->right == NULL))
    {
        return root;
    }

    if (root->key <= key)
    {
        return tree_search_closest_match(root->right, key);
    }

    return tree_search_closest_match(root->left, key);
}

/* Return tree node by the given key, if there is no exact match (i.e.
 * key not found), return NULL. */
static tree_node_t *tree_search(tree_node_t *root, unsigned key)
{
    tree_node_t *result = tree_search_closest_match(root, key);
    if ((result != NULL) && (result->key == key))
    {
        return result;
    }
    return NULL;
}

/* Helper function for array sorting. */
static void swap_elements(pair_t *p1, pair_t *p2)
{
	pair_t tmp = *p1;
    *p1 = *p2;
    *p2 = tmp;
}

/* Simple bubble sort to get our gear arrays in order. */
static void sort_array_by_key(pair_t array[], size_t length)
{
    int i, j;
    for (i = 0; i < length - 1; i++)
    {
        for (j = 0; j < length - i - 1; j++)
        {
            if (array[j].key > array[j + 1].key)
            {
                swap_elements(&(array[j]), &(array[j + 1]));
            }
        }
    }
}

static pair_t *select_gear_from_rpm(tree_node_t *tree, float rpm)
{
    tree_node_t *result;

    /* handle two cases that do not need extra searching */
    if (rpm <= 0)
    {
        return &(mh400e_gears[MH400E_NEUTRAL_GEAR_INDEX]);
    }
    else if (rpm >= MH400E_MAX_RPM)
    {
        return &(mh400e_gears[MH400E_MAX_GEAR_INDEX]);
    }
    else if ((rpm > 0) && (rpm <= mh400e_gears[MH400E_MIN_RPM_INDEX].key))
    {
        /* special case: everything >0 but lower than the lowest gear
         * should still return the lowest gear, because >0 means we want
         * the spindle to rotate */
        return &(mh400e_gears[MH400E_MIN_RPM_INDEX]);
    }

    result = tree_search_closest_match(tree, (unsigned)round(rpm));

    return &(mh400e_gears[result->value]);
}

