#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <BridgeClient.h>
#include <Bridge.h>

#include "mqtt.h"

IPAddress server(192, 168, 0, 100);
MQTT mqtt = MQTT(server, 1883);

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

adafruit_bno055_offsets_t sensor_calib = {
	.accel_offset_x = -8,
	.accel_offset_y = (uint16_t) 8,
	.accel_offset_z = (uint16_t) 18,
	.gyro_offset_x = 0,
	.gyro_offset_y = 0,
	.gyro_offset_z = 0,
	.mag_offset_x = 0,
	.mag_offset_y = 0,
	.mag_offset_z = 0,
	.accel_radius = 0,
	.mag_radius = 0
};

// note: the _in array should have increasing values
int multiMap(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

int in_analogRead[] = { 190, 315, 390, 460, 540, 600 };
int out_pressure[]  = {   0,  40,  60,  80, 100, 120 };

void subscribe()
{
  /* Subscribe to nothing */
}

void setup()
{
  Serial.begin(9600);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  /*flash onboard LED for debug*/
  Serial.println();
  Serial.println();
  Serial.println();
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(13, HIGH);
    Serial.print(F("."));
    if (i % 10 == 0)
    {
      Serial.println();
    }
    delay(10);
    digitalWrite(13, LOW);
    delay(100);
  }
  Serial.println();
  delay(50);

  mqtt.init();
  mqtt.set_subscribe_callback(subscribe);

  mqtt.loop();
  mqtt.debug("imu (" __DATE__ " " __TIME__ ")");
  Serial.println("imu (" __DATE__ " " __TIME__ ")");
  mqtt.loop();

  delay(100);
  /* Initialise the sensor */
  while( !bno.begin() ) {
    mqtt.debug("Error initializing BNO055");
    delay(1000);
  }
  mqtt.debug("BNO055 initialized");

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  /* Load calibration profile */
  bno.setSensorOffsets(sensor_calib);
}


unsigned long last_t = 0;
void loop()
{
  mqtt.loop();

  unsigned long t = millis();
  if (t - last_t >= HEARTBEAT_INTERVAL) {
    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    StaticJsonBuffer<MQTT_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["x"] = linearaccel.x();
    root["y"] = linearaccel.y();
    root["z"] = linearaccel.z();
    root["t"] = t;

    root.printTo(mqtt.stringBuffer, sizeof(mqtt.stringBuffer));
    mqtt.client.publish("sensor/center/accel", mqtt.stringBuffer);

    int pressure_levitation = analogRead(A2);
    int pressure_pneumatics = analogRead(A1);
    JsonObject& p_root = jsonBuffer.createObject();

    p_root["levitation"] = multiMap(pressure_levitation, in_analogRead, out_pressure, sizeof(in_analogRead)/sizeof(int*));
    p_root["pneumatics"] = multiMap(pressure_pneumatics, in_analogRead, out_pressure, sizeof(in_analogRead)/sizeof(int*));
		p_root["t"] = t;
    p_root.printTo(mqtt.stringBuffer, sizeof(mqtt.stringBuffer));
    mqtt.client.publish("sensor/all/pressure", mqtt.stringBuffer);

    last_t = t;
  }
}
