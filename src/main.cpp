#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(115200);
    Serial.println("16 channel PWM test!");

    // forward    13.81ms
    // stop       13.35ms
    // backwardsa 12.96ms

    pwm.begin();
    /*
     * In theory the internal oscillator (clock) is 25MHz but it really isn't
     * that precise. You can 'calibrate' this by tweaking this number until
     * you get the PWM update frequency you're expecting!
     * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     * is used for calculating things like writeMicroseconds()
     * Analog servos run at ~50 Hz updates, It is importaint to use an
     * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
     *    the I2C PCA9685 chip you are setting the value for.
     * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
     *    expected value (50Hz for most ESCs)
     * Setting the value here is specific to each individual I2C PCA9685 chip and
     * affects the calculations for the PWM update frequency.
     * Failure to correctly set the int.osc value will cause unexpected PWM results
     */
    pwm.setOscillatorFrequency(25400000);
    pwm.setPWMFreq(67);  // This is the maximum PWM frequency

    // // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
    // // some i2c devices dont like this so much so if you're sharing the bus, watch
    // // out for this!
    Wire.setClock(400000);

    pinMode(LED_BUILTIN, OUTPUT);
    delay(10);

    for (uint16_t i = 0; i < 16; i += 1) {
        pwm.setPWM(i, 3644, 0);
    }

    delay(100);
}

void loop() {
    for (uint16_t i = 0; i < 16; i += 1) {
        digitalWrite(LED_BUILTIN, HIGH);
        // pwm.setPWM(i, 3770, 0);  // max forward
        // pwm.setPWM(i, 3644, 0); // stop
        pwm.setPWM(i, 3530, 0);  // max backward
        delay(30);
        digitalWrite(LED_BUILTIN, LOW);
        delay(20);
    }
}
