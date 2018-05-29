#include "ble_config.h"

/*
 * Provides skeleton code to interact with the Android FaceTrackerBLE app 
 * 
 * Created by Jon Froehlich, May 7, 2018
 * 
 * Based on previous code by Liang He, Bjoern Hartmann, 
 * Chris Dziemborowicz and the RedBear Team. See: 
 * https://github.com/jonfroehlich/CSE590Sp2018/tree/master/A03-BLEAdvanced
 */

SYSTEM_MODE(SEMI_AUTOMATIC); 

#define RECEIVE_MAX_LEN  5 // TODO: change this based on how much data you are sending from Android 
#define SEND_MAX_LEN    4

// Must be an integer between 1 and 9 and and must also be set to len(BLE_SHORT_NAME) + 1
#define BLE_SHORT_NAME_LEN 8 

// The number of chars should be BLE_SHORT_NAME_LEN - 1. So, for example, if your BLE_SHORT_NAME was 'J', 'o', 'n'
// then BLE_SHORT_NAME_LEN should be 4. If 'M','a','k','e','L','a','b' then BLE_SHORT_NAME_LEN should be 8
// TODO: you must change this name. Otherwise, you will not be able to differentiate your RedBear Duo BLE
// device from everyone else's device in class.
#define BLE_SHORT_NAME 'R','I','S','D','e','m','o'  

/* Define the pins on the Duo board
 * TODO: change and add/subtract the pins here for your applications (as necessary)
 */
#define LEFT_EYE_ANALOG_OUT_PIN D4
#define RIGHT_EYE_ANALOG_OUT_PIN D5
#define HAPPINESS_ANALOG_OUT_PIN D6

const int TRIG_PIN = D0;
const int ECHO_PIN = D1;
const int PIEZO_PIN = D9;
const int LED_PIN = D12;
const int SERVO_OUTPUT_PIN = D2;
const unsigned int MAX_DIST = 23200;
const int SMOOTHING_WINDOW_SIZE = 3;
unsigned long t1;
unsigned long t2;
unsigned long pulse_width;
int cm;
float inches;
bool startSendingWindow = false;

#define MAX_SERVO_ANGLE  180
#define MIN_SERVO_ANGLE  0

#define BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN D7

// happiness meter (servo)
Servo _happinessServo;
Servo _faceAngle;

// Device connected and disconnected callbacks
void deviceConnectedCallback(BLEStatus_t status, uint16_t handle);
void deviceDisconnectedCallback(uint16_t handle);

// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_tx_uuid[16] = { 0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_rx_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve
static uint16_t send_handle = 0x0000; // send

static uint firstWindowValue = 0;
static uint sumOfWindow = 0;
static uint countWindow = 0;
static uint8_t WindowValues[SMOOTHING_WINDOW_SIZE] = { 0x00 };

static uint8_t receive_data[RECEIVE_MAX_LEN] = { 0x01 };
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size); // function declaration for receiving data callback
static uint8_t send_data[SEND_MAX_LEN] = { 0x00 };

// Define the configuration data
static uint8_t adv_data[] = {
  0x02,
  BLE_GAP_AD_TYPE_FLAGS,
  BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE, 
  
  BLE_SHORT_NAME_LEN,
  BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
  BLE_SHORT_NAME, 
  
  0x11,
  BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
  0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71 
};

static btstack_timer_source_t send_characteristic;
static void bleSendDataTimerCallback(btstack_timer_source_t *ts); // function declaration for sending data callback
int _sendDataFrequency = 200; // 200ms (how often to read the pins and transmit the data to Android)

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Face Tracker BLE Demo.");

  // Initialize ble_stack.
  ble.init();
  
  // Register BLE callback functions
  ble.onConnectedCallback(bleConnectedCallback);
  ble.onDisconnectedCallback(bleDisconnectedCallback);

  //lots of standard initialization hidden in here - see ble_config.cpp
  configureBLE(); 
  
  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);
  
  // Register BLE callback functions
  ble.onDataWriteCallback(bleReceiveDataCallback);

  // Add user defined service and characteristics
  ble.addService(service1_uuid);
  receive_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, receive_data, RECEIVE_MAX_LEN);
  send_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, send_data, SEND_MAX_LEN);

  // BLE peripheral starts advertising now.
  ble.startAdvertising();
  Serial.println("BLE start advertising.");

  // Setup pins
  pinMode(LEFT_EYE_ANALOG_OUT_PIN, OUTPUT);
  pinMode(RIGHT_EYE_ANALOG_OUT_PIN, OUTPUT);
  pinMode(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, OUTPUT);
  pinMode(SERVO_OUTPUT_PIN, OUTPUT);
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  _happinessServo.attach(HAPPINESS_ANALOG_OUT_PIN);
  _happinessServo.write( (int)((MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / 2.0) );

  _faceAngle.attach(SERVO_OUTPUT_PIN);
  _faceAngle.write( (int)((MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / 2.0) );


  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Start a task to check status of the pins on your RedBear Duo
  // Works by polling every X milliseconds where X is _sendDataFrequency
  send_characteristic.process = &bleSendDataTimerCallback;
  ble.setTimer(&send_characteristic, _sendDataFrequency); 
  ble.addTimer(&send_characteristic);
}

void loop() 
{
  // Not currently used. The "meat" of the program is in the callback bleWriteCallback and send_notify
}

/**
 * @brief Connect handle.
 *
 * @param[in]  status   BLE_STATUS_CONNECTION_ERROR or BLE_STATUS_OK.
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleConnectedCallback(BLEStatus_t status, uint16_t handle) {
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("BLE device connected!");
      digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, HIGH);
      break;
    default: break;
  }
}

/**
 * @brief Disconnect handle.
 *
 * @param[in]  handle   Connect handle.
 *
 * @retval None
 */
void bleDisconnectedCallback(uint16_t handle) {
  Serial.println("BLE device disconnected.");
  digitalWrite(BLE_DEVICE_CONNECTED_DIGITAL_OUT_PIN, LOW);
}

/**
 * @brief Callback for receiving data from Android (or whatever device you're connected to).
 *
 * @param[in]  value_handle  
 * @param[in]  *buffer       The buffer pointer of writting data.
 * @param[in]  size          The length of writting data.   
 *
 * @retval 
 */
int bleReceiveDataCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {

  if (receive_handle == value_handle) {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    Serial.print("Received data: ");
    for (uint8_t index = 0; index < RECEIVE_MAX_LEN; index++) {
      Serial.print(receive_data[index]);
      Serial.print(" ");
    }
    Serial.println(" ");
    
    // process the data. 
    if (receive_data[0] == 0x01) { //receive the face data 
      // CSE590 Student TODO
      // Write code here that processes the FaceTrackerBLE data from Android
      // and properly angles the servo + ultrasonic sensor towards the face
      // Example servo code here: https://github.com/jonfroehlich/CSE590Sp2018/tree/master/L06-Arduino/RedBearDuoServoSweep  

        int value = receive_data[1] << 24 | (receive_data[2] & 0xFF) << 16 | (receive_data[3] & 0xFF) << 8 | (receive_data[4] & 0xFF);
        Serial.println(value);
        //analogWrite(SERVO_OUTPUT_PIN, 255);
        if (value > 180)
        {
          value = 180;
        }
        if (value < 0)
        {
          value = 0;
        }
        _faceAngle.write(value);

    }
    if (receive_data[0] == 0x02) { //receive the face data 
      Serial.println("Ringing Alarm");
      tone(PIEZO_PIN, 1000);
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      noTone(PIEZO_PIN);
      digitalWrite(LED_PIN, LOW);
      delay(1000);  
      
    }
  }
  return 0;
}

/**
 * @brief Timer task for sending status change to client.
 * @param[in]  *ts   
 * @retval None
 * 
 * Send the data from either analog read or digital read back to 
 * the connected BLE device (e.g., Android)
 */
static void bleSendDataTimerCallback(btstack_timer_source_t *ts) {
  // CSE590 Student TODO
  // Write code that uses the ultrasonic sensor and transmits this to Android
  // Example ultrasonic code here: https://github.com/jonfroehlich/CSE590Sp2018/tree/master/L06-Arduino/RedBearDuoUltrasonicRangeFinder
  // Also need to check if distance measurement < threshold and sound alarm
  int threshold = 5000;
// Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println("im here");
  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );
  Serial.println("after echo pin 0");
  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  // TODO: We probably need to check for a timeout here just in case
  // the ECHO_PIN never goes HIGH... so like
  // while ( digitalRead(ECHO_PIN) == 1 && micros() - t1 < threshold);
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1 && micros() - t1 < threshold);
  
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  // of sound in air at sea level (~340 m/s).
  // Datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
  cm = pulse_width / 58.0;

  Serial.print("CM = ");
  Serial.println(cm);
  
  WindowValues[countWindow] = cm;

  int i = 0;
  sumOfWindow = 0;
  while ( i < SMOOTHING_WINDOW_SIZE)
  {
    sumOfWindow = sumOfWindow + WindowValues[i];
    i = i + 1;
  }
  
  countWindow = countWindow + 1;
  if (countWindow == SMOOTHING_WINDOW_SIZE)
  {
    startSendingWindow = true;  
    countWindow = 0;  
  }

  if (startSendingWindow == true)
  {
    cm = sumOfWindow/SMOOTHING_WINDOW_SIZE;
    
    // Print out results
    if ( pulse_width > MAX_DIST ) {
      Serial.println("Out of range");
    } else {
      Serial.print(cm);
      Serial.print(" cm \t");
    }

    send_data[0] = (byte) (cm >> 24);
    send_data[1] = (byte) (cm >> 16);
    send_data[2] = (byte) (cm >> 8);
    send_data[3] = (byte) (cm /*>> 0*/);
    ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);

    if (cm <= 50)
    {    
      tone(PIEZO_PIN, 1000);
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      noTone(PIEZO_PIN);
      digitalWrite(LED_PIN, LOW);
      delay(1000);  
      
    }
  }
  ble.setTimer(ts, _sendDataFrequency);
  ble.addTimer(ts);
}
