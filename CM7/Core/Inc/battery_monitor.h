#ifndef __BATTERY_MONITOR_H
#define __BATTERY_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

void BatteryMonitor_Init(void);
float BatteryMonitor_ReadVoltage_V(void);

#ifdef __cplusplus
}
#endif

#endif
