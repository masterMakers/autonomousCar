
#include <Encoder.h>

Encoder motorEnc(2, 3);
long motorTicksPrev;
unsigned long lastCompute = 0;
const double K = 1000.0 / (240 / (0.1524 * 3.14159)); // (ms/s) / (ticksPerMeter)
                           // ticksPerMeter = ticksPerRev / MetersPerRev

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    motorTicksPrev = motorEnc.read();
    unsigned long now = millis();
    lastCompute = now;
}

void loop()
{
    // velocity calculation
    long motorTicks = motorEnc.read();
    unsigned long now = millis();
    double dt = double(now - lastCompute);
    lastCompute = now;

    double speed = double(motorTicks - motorTicksPrev) * K / dt;
    motorTicksPrev = motorTicks;

    Serial.println(speed);
    delay(10);
}

