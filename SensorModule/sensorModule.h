#ifndef SENSOR_MODULE_H
#define SENSOR_MODULE_H

void* sensorModule(void* id);

// Initialize Sensor Module
void initSensorModule();

// Stop Sensor Module
void stopSensorModule();

int isSensorModuleRunning(void);
int isSensorModuleReady(void);
const char* getSensorModuleStatus(void);
#endif // SENSOR_MODULE_H
