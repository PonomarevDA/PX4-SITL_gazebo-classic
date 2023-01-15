
MAVLink interface is:

- void SendGpsMessages(const SensorData::Gps &data);
- void UpdateBarometer(const SensorData::Barometer &data, const int id = 0);
- void UpdateAirspeed(const SensorData::Airspeed &data, const int id = 0);
- void UpdateIMU(const SensorData::Imu &data, const int id = 0);
- void UpdateMag(const SensorData::Magnetometer &data, const int id = 0);

