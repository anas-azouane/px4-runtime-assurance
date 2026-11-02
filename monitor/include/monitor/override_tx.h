#ifndef MONITOR_OVERRIDE_TX_H
#define MONITOR_OVERRIDE_TX_H

#include <stdint.h>

#include "monitor/types.h"

void override_tx_command(enum action action, uint8_t confirmation);
void override_tx_heartbeat(void);
/* NAMED_VALUE_INT "MON_ACT": action | rule mask << 8, stamped with this core's clock. */
void override_tx_report(enum action action, uint8_t rule_mask, uint32_t now);

#endif
