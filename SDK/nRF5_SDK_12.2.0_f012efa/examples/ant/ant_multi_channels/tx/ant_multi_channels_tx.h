/*
This software is subject to the license described in the license.txt file included with this software distribution.
You may not use this file except in compliance with this license.
Copyright ? Dynastream Innovations Inc. 2015
All rights reserved.
*/

#ifndef ANT_SCALEABLE_TX_H__
#define ANT_SCALEABLE_TX_H__

#include <stdint.h>
#include "ant_stack_handler_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Function for setting up and opening channels to be ready for TX broadcast.
 */
void ant_scaleable_channel_tx_broadcast_setup(void);

/**@brief Function for handling ANT TX channel events.
 *
 * @param[in] p_ant_evt A pointer to the received ANT event to handle.
 */
void ant_scaleable_event_handler(ant_evt_t * p_ant_evt);


#ifdef __cplusplus
}
#endif

#endif // ANT_SCALEABLE_TX_H__
