#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESP32Encoder.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <Wire.h>

ESP32Encoder encoder_left;
ESP32Encoder encoder_right;
int MOTOR_IDX_LINEAR = 15;
int MOTOR_IDX_ANGULAR = 12;

// right angular limit  3620
// left angular limit   3820

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

AsyncWebServer server(80);

const char *ssid = "Verlab";
const char *password = "Verlab$Router";
String cmd_linear_val = "3644";
String cmd_angular_val = "2048";
bool led_state = true;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP Web Server</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.3rem;}
    p {font-size: 1.9rem;}
    body {max-width: 400px; margin:0px auto; padding-bottom: 25px;}
    .slider { -webkit-appearance: none; margin: 14px; width: 360px; height: 25px; background: #FFD65C;
      outline: none; -webkit-transition: .2s; transition: opacity .2s;}
    .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 35px; background: #003249; cursor: pointer;}
    .slider::-moz-range-thumb { width: 35px; height: 35px; background: #003249; cursor: pointer; } 
  </style>
</head>
<body>
  <h2>ESP Web Server</h2>
  <p><span id="textpwmSlider1">Linear cmd (PWM): %SLIDEVAL1%</span></p>
  <p><input type="range" onchange="updateSliderPWM(this)" 
        id="pwmSlider1" min="3560" max="4096" value="%SLIDEVAL1%" step="1" class="slider"></p>

    <p><span id="textpwmSlider2">Angular cmd (PWM): %SLIDEVAL2%</span></p>
  <p><input type="range" onchange="updateSliderPWM(this)" 
        id="pwmSlider2" min="0" max="4096" value="%SLIDEVAL2%" step="1" class="slider"></p>
<script>
function updateSliderPWM(element) {
  var sliderValue = element.value;
  var sliderId = element.id;
  var labelSliderId = "text" + sliderId;

  console.log(element.id + "=" + sliderValue);
  document.getElementById("text" + sliderId).innerHTML = sliderValue;

  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/slider?" + sliderId + "=" + sliderValue, true);
  xhr.send();
}
</script>
</body>
</html>
)rawliteral";

String processor(const String &var) {
    if (var == "SLIDEVAL1") {
        return cmd_linear_val;
    } else if (var == "SLIDEVAL2") {
        return cmd_angular_val;
    }

    return String();
}

void setup() {
    Serial.begin(115200);

    //  Enable the weak pull up resistors
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder_left.attachHalfQuad(15, 16);
    encoder_right.attachHalfQuad(18, 17);

    encoder_left.clearCount();
    encoder_right.clearCount();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: http://");
    Serial.println(WiFi.localIP());

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html, processor);
    });

    // Send a GET request to <ESP_IP>/slider?<param>=<inputMessage>
    server.on("/slider", HTTP_GET, [](AsyncWebServerRequest *request) {
        String inputMessage;
        // GET input value on <ESP_IP>/slider?<param>=<inputMessage>
        if (request->hasParam("pwmSlider1")) {
            inputMessage = request->getParam("pwmSlider1")->value();
            cmd_linear_val = inputMessage;
            pwm.setPWM(MOTOR_IDX_LINEAR, cmd_linear_val.toInt(), 0);
            Serial.println("Updated pwmSlider1:" + inputMessage);
        } else if (request->hasParam("pwmSlider2")) {
            inputMessage = request->getParam("pwmSlider2")->value();
            cmd_angular_val = inputMessage;
            pwm.setPWM(MOTOR_IDX_ANGULAR, cmd_angular_val.toInt(), 0);
            Serial.println("Updated pwmSlider2:" + inputMessage);
        }
        request->send(200, "text/plain", "OK");
    });

    server.begin();
    Serial.println("16 channel PWM test!");

    /*
     * In theory the internal oscillator (clock) is 25MHz but it really isn't
     * that precise. You can 'calibrate' this by tweaking this number until
     * you get the PWM update frequency you're expecting (using an oscilloscope)!
     * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
     *    the I2C PCA9685 chip you are setting the value for.
     * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
     *    expected value (50Hz for most ESCs)
     */
    bool res = Wire.begin(6, 7);
    pwm.begin();
    pwm.setOscillatorFrequency(25400000);
    pwm.setPWMFreq(67);

    pinMode(LED_BUILTIN, OUTPUT);

    pwm.setPWM(MOTOR_IDX_LINEAR, 3643, 0);
    pwm.setPWM(MOTOR_IDX_ANGULAR, 2048, 0);

    Serial.println("Starting...");
}

void loop() {
    // TODOS:
    // Add battery meter
    // Add way to know if the PWM board was correctly initialized and connected
    // Add way to know if the encoders are working OK
    // Add limit in speed, and P controller to prevent sudden increases in speed (rampup curve)
    // Add automatic code for stop and reverse
    // Convert PWM to linear speed in m/s
    // Convert PWM into angle (degrees)
    // Calibrate odometry

    // blink big led
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state);

    double left_rpm = encoder_left.getRPM();
    double right_rpm = encoder_right.getRPM();
    long double avg_rpm = (left_rpm + right_rpm) / 2.0;
    long double avg_odom = (encoder_left.getOdometry() + encoder_right.getOdometry()) / 2.0;
    float avg_linearspeed = (encoder_left.getLinearSpeed() + encoder_right.getLinearSpeed()) / 2.0;

    Serial.println("avg.odom(cm):" + String((double)avg_odom) + "\tavg.lin(m/s):" + String(avg_linearspeed) + "\tlRPM:" + String(left_rpm) + "\trRPM:" + String(right_rpm));
    delay(100);
}