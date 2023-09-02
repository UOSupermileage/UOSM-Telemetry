#include "ArduinoIoTCloud.h"
#include "Arduino_ConnectionHandler.h"
#include "ApplicationTypes.h"

const char SSID[] = ""; 
const char PASSWORD[] = ""; 
const char  DEVICE_KEY[] = "";  
const char DEVICE_ID[] = ""; 
float new_accel_x; 
float new_accel_y; 
float new_accel_z; 
void initProperties(){ 
    ArduinoCloud.setThingId(DEVICE_ID); 
    ArduinoCloud.setThingId(DEVICE_KEY); 
    ArduinoCloud.addProperty(new_accel_x,READ, ON_CHANGE); 
    ArduinoCloud.addProperty(new_accel_y,READ,ON_CHANGE); 
    ArduinoCloud.addProperty(new_accel_z,READ,ON_CHANGE); 

}

void updateAcceleration(accel_t acceleration_struct){ 
    new_accel_x = acceleration_struct.accel_x; 
    new_accel_y = acceleration_struct.accel_y; 
    new_accel_z = acceleration_struct.accel_z; 
}

WiFiConnectionHandler AIoTPreferredConnection(SSID,PASSWORD); 
