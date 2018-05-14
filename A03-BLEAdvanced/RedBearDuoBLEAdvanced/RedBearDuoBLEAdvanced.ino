#include "ble_config.h"

/*
 * Simple Bluetooth Demo
 * This code demonstrates how the RedBear Duo board communicates with 
 * the paired Android app. The user can use the Android app to 
 * do digital read & write, analog read & write, and servo control.
 * Created by Liang He, April 27th, 2018
 * Updated by Jon Froehlich, May 8, 2018
 * 
 * The Library is created based on Bjorn's code for RedBear BLE communication: 
 * https://github.com/bjo3rn/idd-examples/tree/master/redbearduo/examples/ble_led
 * 
 * Our code is created based on the provided example code (Simple Controls) by the RedBear Team:
 * https://github.com/RedBearLab/Android
 */

#if defined(ARDUINO) 
SYSTEM_MODE(SEMI_AUTOMATIC); 
#endif

#define RECEIVE_MAX_LEN    4
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
#define DIGITAL_OUT_PIN            D0
#define RED_OUT_PIN                A0
#define BLUE_OUT_PIN               A1
#define GREEN_OUT_PIN              A6
#define DIGITAL_IN_PIN             D12
#define PWM_PIN                    D3
#define SERVO_PIN                  D4
#define ANALOG_IN_PIN              A5

// Servo declaration
Servo myservo;

byte redValue = 0;
byte blueValue = 0;
byte greenValue = 0;
const int SMOOTHING_WINDOW_SIZE = 10;
int smoothing_counter = 0;
int smoothing_red = 0;
int smoothing_green = 0;
int smoothing_blue = 0;

// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_tx_uuid[16] = { 0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_rx_uuid[16] = { 0x71,0x3d,0x00,0x02,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve
static uint16_t send_handle = 0x0000; // send

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

// Mark whether need to notify analog value to client.
static boolean analog_enabled = false;

// Input pin state.
static byte old_state = LOW;

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Simple Controls demo.");

  // Initialize ble_stack.
  ble.init();
  configureBLE(); //lots of standard initialization hidden in here - see ble_config.cpp
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

  /*
   * TODO: This is where you can initialize all peripheral/pin modes
   */
  pinMode(DIGITAL_OUT_PIN, OUTPUT);
  pinMode(DIGITAL_IN_PIN, INPUT_PULLDOWN);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(RED_OUT_PIN, OUTPUT);
  pinMode(BLUE_OUT_PIN, OUTPUT);
  pinMode(GREEN_OUT_PIN, OUTPUT);
  
  // Initial the servo as always with Arduino board
  myservo.attach(SERVO_PIN);
  myservo.write(0);

  // Start a task to check status of the pins on your RedBear Duo
  // Works by polling every X milliseconds where X is currently 500
  send_characteristic.process = &bleSendDataTimerCallback;
  ble.setTimer(&send_characteristic, _sendDataFrequency); 
  ble.addTimer(&send_characteristic);
}

void loop() 
{
  // Not currently used. The "meat" of the program is in the callback bleWriteCallback and send_notify
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
  Serial.print("Write value handler: ");
  Serial.println(value_handle, HEX);

  if (receive_handle == value_handle) {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    Serial.print("Write value: ");
    for (uint8_t index = 0; index < RECEIVE_MAX_LEN; index++) {
      Serial.print(receive_data[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
    
    /* Process the data
     * TODO: Receive the data sent from other BLE-abled devices (e.g., Android app)
     * and process the data for different purposes (digital write, digital read, analog read, PWM write)
     */
    if (receive_data[0] == 0x01) { // Command is to control digital out pin
      if (receive_data[1] == 0x01)
      {
        digitalWrite(DIGITAL_OUT_PIN, HIGH);
        analogWrite(RED_OUT_PIN, 255);
        analogWrite(GREEN_OUT_PIN, 0);
        analogWrite(BLUE_OUT_PIN, 0);
      }
      else
      {
        digitalWrite(DIGITAL_OUT_PIN, LOW);
        analogWrite(RED_OUT_PIN, 0);
        analogWrite(GREEN_OUT_PIN, 255);
        analogWrite(BLUE_OUT_PIN, 0);
      }
    }
    else if (receive_data[0] == 0xA0) { // Command is to enable analog in reading
      if (receive_data[1] == 0x01)
        analog_enabled = true;
      else
        analog_enabled = false;
    }
    else if (receive_data[0] == 0x02) { // Command is to control PWM pin
      analogWrite(PWM_PIN, receive_data[1]);
    }
    else if (receive_data[0] == 0x03) { // Command is to control Servo pin
      myservo.write(receive_data[1]);
    }
    else if (receive_data[0] == 0x04) { // Command is to initialize all.
      analog_enabled = false;
      myservo.write(0);
      analogWrite(PWM_PIN, 0);
      digitalWrite(DIGITAL_OUT_PIN, LOW);
      old_state = LOW;
    }
    else if (receive_data[0] == 0x05) { // Manual color input or from stepcount.
      analog_enabled = false;
      Serial.print("RGB Values:  ");
      Serial.print(receive_data[1]);
      redValue = receive_data[1];
      analogWrite(RED_OUT_PIN, 255 - receive_data[1]);
      Serial.print(", ");
      greenValue = receive_data[2];
      Serial.print(receive_data[2]);
      analogWrite(GREEN_OUT_PIN, 255 - receive_data[2]);
      Serial.print(", ");      
      blueValue = receive_data[3];
      Serial.println(receive_data[3]);
      analogWrite(BLUE_OUT_PIN, 255 - receive_data[3]);
    }
    else if (receive_data[0] == 0x06) { // Needs smoothing, from shake
      analog_enabled = false;
      if (smoothing_counter < SMOOTHING_WINDOW_SIZE)
      {
        smoothing_red = smoothing_red + receive_data[1];
        smoothing_green = smoothing_green + receive_data[2];
        smoothing_blue = smoothing_red + receive_data[3];
      }
      else
      {
        smoothing_counter = 0;
        redValue = smoothing_red/SMOOTHING_WINDOW_SIZE;
        greenValue = smoothing_green/SMOOTHING_WINDOW_SIZE;
        blueValue = smoothing_blue/SMOOTHING_WINDOW_SIZE;
      
      Serial.print("RGB Values:  ");
      Serial.print(redValue);
      analogWrite(RED_OUT_PIN, 255 - redValue);
      Serial.print(", ");
      
      Serial.print(greenValue);
      analogWrite(GREEN_OUT_PIN, 255 - greenValue);
      Serial.print(", ");      
      
      Serial.println(blueValue);
      analogWrite(BLUE_OUT_PIN, 255 - blueValue);
      }
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
  if (analog_enabled) { // if analog reading enabled.
    //Serial.println("characteristic2_notify analog reading ");
    // Read and send out
    uint16_t value = analogRead(ANALOG_IN_PIN);
    send_data[0] = redValue;
    send_data[1] = greenValue;
    send_data[2] = blueValue;
    Serial.print("I'm printing analog values of rgb 1: ");
    Serial.print(redValue);
    Serial.print(", ");
    Serial.print(greenValue);
    Serial.print(", ");
    Serial.print(blueValue);
    Serial.println();
    if (ble.attServerCanSendPacket())
      ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
  }
  
  // If digital in changes, report the state.
  if (digitalRead(DIGITAL_IN_PIN) != old_state) {
    Serial.println("send_notify digital reading ");
    old_state = digitalRead(DIGITAL_IN_PIN);
    if (digitalRead(DIGITAL_IN_PIN) == HIGH) {
      send_data[0] = (0x0A);
      send_data[1] = (0x01);
      send_data[2] = (0x00);
      ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
    }
    else {
    send_data[0] = redValue;
    send_data[1] = greenValue;
    send_data[2] = blueValue;
    Serial.println("I'm printing analog values of rgb 2");
      ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
    }
  }
  // Restart timer.
  ble.setTimer(ts, _sendDataFrequency);
  ble.addTimer(ts);
}
