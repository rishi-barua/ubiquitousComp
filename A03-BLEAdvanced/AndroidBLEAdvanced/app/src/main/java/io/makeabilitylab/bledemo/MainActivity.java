/* BLE demo: Demonstrate how to bi-directionally communicate with the Duo board through BLE. In this
 * example code, it shows how to send digital and analog data from the Android app to the Duo board
 * and how to receive data from the board.
 *
 * The app is built based on the example code provided by the RedBear Team:
 * https://github.com/RedBearLab/Android
 */

package io.makeabilitylab.bledemo;

import android.content.Intent;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.support.annotation.NonNull;
import android.support.constraint.ConstraintLayout;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.jjoe64.graphview.series.DataPoint;
import com.jjoe64.graphview.series.LineGraphSeries;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

import io.makeabilitylab.bledemo.ble.BLEDevice;
import io.makeabilitylab.bledemo.ble.BLEListener;
import io.makeabilitylab.bledemo.ble.BLEUtil;
import yuku.ambilwarna.AmbilWarnaDialog;

public class MainActivity extends AppCompatActivity implements BLEListener, SensorEventListener {

    // TODO: Define your device name and the length of the name. For your assignment, do not use the
    // default name or you will not be able to discriminate your board from everyone else's board.
    // Note the device name and the length should be consistent with the ones defined in the Duo sketch
    private final String TARGET_DEVICE_NAME = "RISDemo";

    // Declare all variables associated with the UI components
    private Button mConnectBtn = null;
    private TextView mDeviceName = null;
    private TextView mRssiValue = null;
    private TextView mUUID = null;
    private TextView mAnalogInValue = null;
    private EditText mStepTrack = null;
    private ToggleButton mDigitalOutBtn, mDigitalInBtn, mAnalogInBtn;
    private SeekBar mServoSeekBar, mPWMSeekBar;

    private LinearLayout lLayout;
    private ConstraintLayout mLayout;
    int mDefaultColor;
    Button mButton;

    private final Handler _handler = new Handler();
    boolean shake = false;
    double acceleration;
    private double[] accValues;
    int stepCount = 0;
    int stepCounterStep = 0;
    int stepCounterInit = 0;
    int currentPointer = 0;
    int timer = 0;
    boolean start = false;
    double smooth_acc = 0;
    double prev_acc = 0;
    double acc_mod = 0;
    long seconds = 0;
    int window_size = 5;
    double threshold = 0;
    boolean set = false;

    private Runnable _timer2;
    private LineGraphSeries<DataPoint> _series2;
    private double _graph2LastXValue = 0d;

    private enum Colors
    {
        Red,
        Green,
        Blue,
        Gray,
        Magenta,
        Yellow
    }
    private SensorManager sensorManager;
    private Sensor mSensor;

    private Colors color = Colors.Blue;

    // Declare all Bluetooth stuff
    private BLEDevice mBLEDevice;

    private void setButtonDisable() {
        mDigitalOutBtn.setEnabled(false);
        mAnalogInBtn.setEnabled(false);
        mServoSeekBar.setEnabled(false);
        mPWMSeekBar.setEnabled(false);
        mConnectBtn.setText("Connect");
        mRssiValue.setText("");
        mDeviceName.setText("");
        mUUID.setText("");
    }

    private void setButtonEnable() {
        mDigitalOutBtn.setEnabled(true);
        mDigitalInBtn.setEnabled(true);
        mAnalogInBtn.setEnabled(true);
        mServoSeekBar.setEnabled(true);
        mPWMSeekBar.setEnabled(true);
        mConnectBtn.setText("Disconnect");
    }

    // Display the received RSSI on the interface
    private void displayData(int rssi) {
        mRssiValue.setText(String.format(Locale.US, "%d", rssi));
        mDeviceName.setText(mBLEDevice.getName());
        mUUID.setText(mBLEDevice.getUUID().toString());
    }

    // Display the received Analog/Digital read on the interface
    private void readAnalogInValue(byte[] data) {
        /*for (int i = 0; i < data.length; i += 3) {
            if (data[i] == 0x0A) {
                if (data[i + 1] == 0x01)
                    mDigitalInBtn.setChecked(false);
                else
                    mDigitalInBtn.setChecked(true);
            } else if (data[i] == 0x0B) {
                int value;
                value = ((data[i + 1] << 8) & 0x0000ff00)
                        | (data[i + 2] & 0x000000ff);
                mAnalogInValue.setText(String.format(Locale.US, "%d", value));
            }
        }*/


        int r = data[0];
        int g = data[1];
        int b = data[2];

        if (r < 0)
        {
            r = r + 256;
        }
        if (g < 0)
        {
            g = g + 256;
        }
        if (b < 0)
        {
            b = b + 256;
        }
        mAnalogInValue.setText(r + ", " + g + ", " + b);
        lLayout.setBackgroundColor(Color.rgb(r, g, b));
    }

    public void EnableAnalog()
    {
        byte[] buf = new byte[] { (byte) 0xA0, (byte) 0x00, (byte) 0x00, (byte) 0x00 };
        buf[1] = 0x01;

        mBLEDevice.sendData(buf);
    }
    public void openColorPicker()
    {
        AmbilWarnaDialog colorPicker = new AmbilWarnaDialog(this, mDefaultColor, new AmbilWarnaDialog.OnAmbilWarnaListener() {
            @Override
            public void onCancel(AmbilWarnaDialog dialog) {

            }

            @Override
            public void onOk(AmbilWarnaDialog dialog, int color) {
                mDefaultColor = color;
                mLayout.setBackgroundColor(mDefaultColor);

                byte buf[] = new byte[] { (byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x00 };

                buf[3] = (byte) (mDefaultColor & 0xFF);
                buf[2] = (byte) ((mDefaultColor >> 8) & 0xFF);
                buf[1] = (byte) ((mDefaultColor >> 16) & 0xFF);

                mBLEDevice.sendData(buf);
            }
        });
        colorPicker.show();
    }
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Associate all UI components with variables
        mConnectBtn = findViewById(R.id.connectBtn);
        mDeviceName = findViewById(R.id.deviceName);
        mRssiValue = findViewById(R.id.rssiValue);
        mAnalogInValue = findViewById(R.id.ananlogIn);
        mDigitalOutBtn = findViewById(R.id.DOutBtn);
        mDigitalInBtn = findViewById(R.id.DInBtn);
        mAnalogInBtn = findViewById(R.id.AInBtn);
        mServoSeekBar = findViewById(R.id.ServoSeekBar);
        mPWMSeekBar = findViewById(R.id.PWMSeekBar);
        mUUID = findViewById(R.id.uuidValue);
        mLayout = (ConstraintLayout) findViewById(R.id.layout);
        lLayout = (LinearLayout) findViewById(R.id.InputColor);
        mDefaultColor = ContextCompat.getColor(MainActivity.this, R.color.colorPrimary);
        mButton = (Button) findViewById(R.id.colorPicker);
        mStepTrack = (EditText) findViewById(R.id.StepTrack);

        mButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                openColorPicker();
            }
        });

        accValues = new double[100];
        _series2 = new LineGraphSeries<>();
        sensorManager=(SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener((SensorEventListener) this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_GAME);

        // Connection button click event
        mConnectBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mBLEDevice.getState() == BLEDevice.State.CONNECTED) {
                    mBLEDevice.disconnect();
                } else {
                    //TODO: Ideally we would present the user with a "wait" message until
                    //a connection was made or a timeout occurred... but this is better than nothing for now
                    Toast toast = Toast.makeText(
                            MainActivity.this,
                            "Attempting to connect to '" + TARGET_DEVICE_NAME + "'",
                            Toast.LENGTH_LONG);
                    toast.show();

                    EnableAnalog();
                    mBLEDevice.connect();
                }
            }
        });

        // Send data to Duo board
        // It has three bytes: maker, data value, reserved
        mDigitalOutBtn.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                byte buf[] = new byte[] { (byte) 0x01, (byte) 0x00, (byte) 0x00, (byte) 0x00 };
                if (isChecked)
                    buf[1] = 0x01;
                else
                    buf[1] = 0x00;
                //mBLEDevice.sendData(buf);
                if (start == false) {
                    start = true;
                }
                else{
                    start = false;
                }
            }
        });

        mAnalogInBtn.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                byte[] buf = new byte[] { (byte) 0xA0, (byte) 0x00, (byte) 0x00, (byte) 0x00 };
                if (isChecked)
                    buf[1] = 0x01;
                else
                    buf[1] = 0x00;
                mBLEDevice.sendData(buf);
            }
        });

        mDigitalInBtn.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (shake == true) {
                    shake = false;
                } else {
                    shake = true;
                }
            }

        });

        // Configure the servo Seekbar
        mServoSeekBar.setEnabled(false);
        mServoSeekBar.setMax(180);  // Servo can rotate from 0 to 180 degree
        mServoSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                byte[] buf = new byte[] { (byte) 0x03, (byte) 0x00, (byte) 0x00, (byte) 0x00 };
                buf[1] = (byte) mServoSeekBar.getProgress();
                mBLEDevice.sendData(buf);
            }
        });

        // Configure the PWM Seekbar
        mPWMSeekBar.setEnabled(false);
        mPWMSeekBar.setMax(255);
        mPWMSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                byte[] buf = new byte[] { (byte) 0x02, (byte) 0x00, (byte) 0x00, (byte) 0x00 };
                buf[1] = (byte) mPWMSeekBar.getProgress();
                mBLEDevice.sendData(buf);
            }
        });

        // Make sure that Bluetooth is supported.
        if (!BLEUtil.isSupported(this)) {
            Toast.makeText(this, "Ble not supported", Toast.LENGTH_SHORT)
                    .show();
            finish();
            return;
        }

        // Make sure that we have required permissions.
        if (!BLEUtil.hasPermission(this)) {
            BLEUtil.requestPermission(this);
        }

        // Make sure that Bluetooth is enabled.
        if (!BLEUtil.isBluetoothEnabled(this)) {
            BLEUtil.requestEnableBluetooth(this);
        }

        mBLEDevice = new BLEDevice(this, TARGET_DEVICE_NAME);
        mBLEDevice.addListener(this);
    }

    @Override
    protected void onResume() {
        super.onResume();

        _timer2 = new Runnable() {
            @Override
            public void run() {
                _graph2LastXValue += 0.2d;
                // _series2.resetData(generateData());

                Log.i("smooth_acc: ", "" + smooth_acc);

                if (set == true && start == true)
                {
                    if (DetectPeak(smooth_acc))
                    {
                        Log.i("stepTaken: ", "step taken");

                        ++stepCount;
                        mStepTrack.setText("New Step Taken");
                        mStepTrack.setHighlightColor(Color.GREEN);
                        mStepTrack.setVisibility(View.VISIBLE);
                        SendSignalOnShakeOrStep();
                        EnableAnalog();
                    }
                    else
                    {
                        mStepTrack.setVisibility(View.INVISIBLE);
                    }
                }

                _handler.postDelayed(this, 100);
            }
        };
        _handler.postDelayed(_timer2, 100);

        if (!BLEUtil.isBluetoothEnabled(this)) {
            BLEUtil.requestEnableBluetooth(this);
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        if (mBLEDevice != null) {
            mBLEDevice.disconnect();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // User chose not to enable Bluetooth.
        if (requestCode == BLEUtil.REQUEST_ENABLE_BLUETOOTH
                && !BLEUtil.isBluetoothEnabled(this)) {
            finish();
            return;
        }

        super.onActivityResult(requestCode, resultCode, data);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
        @NonNull int[] grantResults) {
        // User chose not to grant required permissions.
        if (requestCode == BLEUtil.REQUEST_BLUETOOTH_PERMISSIONS
                && !BLEUtil.hasPermission(this)) {
            finish();
            return;
        }

        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    }

    @Override
    public void onBleConnected() {
        Toast.makeText(getApplicationContext(), "Connected", Toast.LENGTH_SHORT).show();
        setButtonEnable();
    }

    @Override
    public void onBleConnectFailed() {
        Toast toast = Toast
                .makeText(
                        MainActivity.this,
                        "Couldn't find the BLE device with name '" + TARGET_DEVICE_NAME + "'!",
                        Toast.LENGTH_SHORT);
        toast.setGravity(0, 0, Gravity.CENTER);
        toast.show();
    }

    @Override
    public void onBleDisconnected() {
        Toast.makeText(getApplicationContext(), "Disconnected", Toast.LENGTH_SHORT).show();
        setButtonDisable();
    }

    @Override
    public void onBleDataReceived(byte[] data) {
        readAnalogInValue(data);
    }

    @Override
    public void onBleRssiChanged(int rssi) {
        displayData(rssi);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        double ax = 0, ay = 0, az = 0;
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {

        }
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

        double[] aVec = new double[3];
        aVec[0] = event.values[0];
        aVec[1] = event.values[1];
        aVec[2] = event.values[2];

            ax = (double) event.values[0];
            ay = (double) event.values[1];
            az = (double) event.values[2];
            acc_mod = vectorMod(aVec);

            if (vectorMod(aVec) > 15) {
                if (shake == true) {
                    SendSignalOnShakeOrStepNeedSmoothing();
                    EnableAnalog();
                }
            }

            Log.i("currentPointer: ", "" + currentPointer);
            if (currentPointer > 30) {
                threshold = DetectThreshold(accValues); // Get threshold. Any threshold less than 9 is not acceptable as g = 9.8m/s2
                if (threshold > 9) {
                    Log.i("set: ", "true");
                    set = true;
                }

                Log.i("Setting threshold = ", "" + threshold);

            }
            accValues[currentPointer] = acc_mod - threshold;

            //Log.i("currentPointer val = ", "" + currentPointer);
            if (currentPointer >= window_size) {
                if (set == true) {
                    prev_acc = acc_mod;
              //      Log.i("prev_acc: ", "" + prev_acc);
                }
                double smooth_val = averageFloat(currentPointer - window_size, accValues, currentPointer); // Smoothen the signal
                smooth_acc = smooth_val;

            } else {
                smooth_acc = acc_mod;
            }

            currentPointer++;
            if (currentPointer == 100) {
                currentPointer = 0;
            }

        }

    }


    public void SendSignalOnShakeOrStep()
    {
        int colorInt = 0;

        if (color == Colors.Red) {
            colorInt = Color.GREEN;
            color = Colors.Green;
        } else if (color == Colors.Green) {
            colorInt = Color.BLUE;
            color = Colors.Blue;
        } else if (color == Colors.Blue) {
            colorInt = Color.GRAY;
            color = Colors.Gray;
        }
        else if (color == Colors.Gray) {
            colorInt = Color.MAGENTA;
            color = Colors.Magenta;
        }
        else if (color == Colors.Magenta) {
            colorInt = Color.YELLOW;
            color = Colors.Yellow;
        }
        else if (color == Colors.Yellow) {
            colorInt = Color.RED;
            color = Colors.Red;
        }


        byte buf[] = new byte[]{(byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x00};

        buf[3] = (byte) (colorInt & 0xFF);
        buf[2] = (byte) ((colorInt >> 8) & 0xFF);
        buf[1] = (byte) ((colorInt >> 16) & 0xFF);
        mBLEDevice.sendData(buf);
    }

    public void SendSignalOnShakeOrStepNeedSmoothing()
    {
        int colorInt = 0;

        if (color == Colors.Red) {
            colorInt = Color.GREEN;
            color = Colors.Green;
        } else if (color == Colors.Green) {
            colorInt = Color.BLUE;
            color = Colors.Blue;
        } else if (color == Colors.Blue) {
            colorInt = Color.GRAY;
            color = Colors.Gray;
        }
        else if (color == Colors.Gray) {
            colorInt = Color.MAGENTA;
            color = Colors.Magenta;
        }
        else if (color == Colors.Magenta) {
            colorInt = Color.YELLOW;
            color = Colors.Yellow;
        }
        else if (color == Colors.Yellow) {
            colorInt = Color.RED;
            color = Colors.Red;
        }


        byte buf[] = new byte[]{(byte) 0x06, (byte) 0x00, (byte) 0x00, (byte) 0x00};

        buf[3] = (byte) (colorInt & 0xFF);
        buf[2] = (byte) ((colorInt >> 8) & 0xFF);
        buf[1] = (byte) ((colorInt >> 16) & 0xFF);
        mBLEDevice.sendData(buf);
    }

    @Override
    public void onPause() {
        _handler.removeCallbacks(_timer2);
        super.onPause();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    public double DetectThreshold(double[] values) {
        double diff = 0;
        Map<Double, Integer> modeDictionary = new HashMap<Double, Integer>();

        for (int i = 1; i < values.length; i++) {
            diff = Math.abs(values[i] - values[i - 1]);
            if (diff < 0.2) {
                boolean updated = false;
                for (Double value : modeDictionary.keySet()) {
                    if (Math.abs(values[i] - value) < 0.2 * value) {
                        updated = true;
                        modeDictionary.put(value, modeDictionary.get(value) + 1);
                        break;
                    }
                }

                if (updated == false) {
                    modeDictionary.put(values[i], 1);
                }
            }
        }

        double highestFreq = 0;
        double threshold = 0;
        for (Double value : modeDictionary.keySet()) {
            if (modeDictionary.get(value) >= highestFreq) {
                threshold = value;
            }
        }

        return threshold;
    }

    private double vectorMod(double[] vec1) // Function to find modulus of a vector
    {
        double val = vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2];
        val = Math.sqrt(val);
        return (val);
    }

    public boolean DetectPeak(double value)
    {
        if (prev_acc > threshold + 3.5)
        {
            Log.i("Ignored noise:", "value : " + prev_acc);
            return false; // noise
        }

        if (value < prev_acc )
        {
            return true;
        }

        return false;
    }

    private double averageFloat(int i, double[] values, int j) // this averages the values in the array within the given window
    {
        double sum = 0;
        for (; i<j; i++)
        {
            sum += values[i];
            // Log.i("values[i] = ", "" + values[i]);
        }

        return sum/window_size;
    }
}
