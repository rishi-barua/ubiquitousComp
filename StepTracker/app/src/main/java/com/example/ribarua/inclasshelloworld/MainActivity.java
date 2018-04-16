package com.example.ribarua.inclasshelloworld;

import android.content.Intent;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.provider.ContactsContract;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.series.DataPoint;
import com.jjoe64.graphview.series.LineGraphSeries;

import java.util.Date;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import static java.lang.System.in;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    // We use timers to intermittently generate random data for the two graphs
    private final Handler _handler = new Handler();
    private Runnable _timer1;
    private Runnable _timer2;

    private LineGraphSeries<DataPoint> _series1;
    private LineGraphSeries<DataPoint> _series2;
    private double _graph2LastXValue = 0d;
    private double _graph1LastXValue = 0d;
    private SensorManager sensorManager;
    private Sensor mSensor;

    private DataPoint[] accelerationSteps;
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
    double ax,ay,az;   // these are the acceleration in x,y and z axis

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        accelerationSteps = new DataPoint[30];

        seconds = (new Date()).getTime();

        sensorManager=(SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener((SensorEventListener) this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener((SensorEventListener) this, sensorManager.getDefaultSensor(Sensor.TYPE_STEP_COUNTER), SensorManager.SENSOR_DELAY_GAME);

        EditText tv = this.findViewById(R.id.newStep);
        tv.setVisibility(View.INVISIBLE);

        accValues = new double[100];
        for (int i = 0; i <30; i++)
        {
            accelerationSteps[i] = new DataPoint(i, 0);
        }

        GraphView graph = (GraphView) this.findViewById(R.id.graph);
        _series1 = new LineGraphSeries<>();
        graph.addSeries(_series1);
        graph.setTitle("Real-Time Graph (Actual)");
        graph.getGridLabelRenderer().setVerticalAxisTitle("Acceleration");
        graph.getGridLabelRenderer().setHorizontalAxisTitle("Seconds");
        graph.getViewport().setXAxisBoundsManual(true);
        graph.getViewport().setMinX(0);
        graph.getViewport().setMaxX(40);
        graph.setVisibility(View.INVISIBLE);

        GraphView graph2 = (GraphView) this.findViewById(R.id.graph2);
        _series2 = new LineGraphSeries<>();
        graph2.setTitle("Real-Time Graph (Smooth)");
        graph2.addSeries(_series2);
        graph2.getGridLabelRenderer().setVerticalAxisTitle("Acceleration");
        graph2.getGridLabelRenderer().setHorizontalAxisTitle("Seconds");
        graph2.getViewport().setXAxisBoundsManual(true);
        graph2.getViewport().setMinX(0);
        graph2.getViewport().setMaxX(40);
        graph2.setVisibility(View.INVISIBLE);

        Button b = findViewById(R.id.showdebug);
        b.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("inside :", "buttonclick");
                Button b = findViewById(R.id.showdebug);
                GraphView graph = findViewById(R.id.graph);
                GraphView graph2 = (GraphView) findViewById(R.id.graph2);
                if (b.getText().toString().equals("Show Debug"))
                {
                    graph.setVisibility(View.VISIBLE);
                    graph2.setVisibility(View.VISIBLE);
                    b.setText("Hide Debug");
                }
                else
                {
                    b.setText("Show Debug");
                    graph.setVisibility(View.INVISIBLE);
                    graph2.setVisibility(View.INVISIBLE);
                }

            }
        });

        Button startButton = findViewById(R.id.startStep);
        startButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("inside :", "startStep");
                Button b = findViewById(R.id.startStep);
                if (b.getText().toString().equals("Start StepCount"))
                {
                    start = true;
                    b.setText("Stop StepCount");
                }
                else
                {
                    start = false;
                    stepCount = 0;
                    stepCounterStep = 0;
                    EditText editText = findViewById(R.id.StepCounter);
                    editText.setText("StepCounter Value = " + stepCounterStep);
                    b.setText("Start StepCount");
                }

            }
        });
    }

    @Override
    public void onResume() {
        super.onResume();
        _timer1 = new Runnable() {
            @Override
            public void run() {
                _graph1LastXValue += 0.2d;
               // _series1.resetData(generateData());


                if (set == true) {
                        _series1.appendData(new DataPoint(_graph1LastXValue, (double) acc_mod), true, 150);
                }
                _handler.postDelayed(this, 200);
            }
        };
        _handler.postDelayed(_timer1, 200);

        _timer2 = new Runnable() {
            @Override
            public void run() {
                _graph2LastXValue += 0.2d;
               // _series2.resetData(generateData());
                Log.i("val", "new data point with value " + smooth_acc);
                //_series2.appendData(new DataPoint(_graph2LastXValue,  smooth_acc), true, 40);


                    if (set == true) {
                        _series2.appendData(new DataPoint(_graph2LastXValue, (double) smooth_acc + threshold), true, 150);
                    }


                if (set == true && start == true)
                {
                    EditText tv = findViewById(R.id.newStep);
                    if (DetectPeak(smooth_acc))
                    {
                        ++stepCount;
                        tv.setText("New Step Taken");
                        tv.setHighlightColor(Color.GREEN);
                        tv.setVisibility(View.VISIBLE);
                    }
                    else
                    {
                        tv.setVisibility(View.INVISIBLE);
                    }
                }

                _handler.postDelayed(this, 200);
            }
        };
        _handler.postDelayed(_timer2, 200);
    }

    @Override
    public void onPause() {
        _handler.removeCallbacks(_timer1);
        _handler.removeCallbacks(_timer2);
        super.onPause();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_STEP_COUNTER && set == true)
        {
            if (stepCounterInit == 0)
            {
                stepCounterInit = (int) event.values[0];
                Log.i("stepCounterSensor def: ", "" + stepCounterInit);
            }
            else
            {
                stepCounterStep = (int) event.values[0] - stepCounterInit;
            }

            Log.i("StepCounter Value : ", "" + stepCounterStep);
            EditText editText = this.findViewById(R.id.StepCounter);
            editText.setText("StepCounter Value = " + stepCounterStep);
        }
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
        {

        }
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            ax = (double) event.values[0];
            ay = (double) event.values[1];
            az = (double) event.values[2];

            double[] aVec = new double[3];
            aVec[0] = ax;
            aVec[1] = ay;
            aVec[2] = az;

            acc_mod = vectorMod(aVec); // Get Mod
            //Log.i("currentPointer = ", "" + currentPointer);
            if (currentPointer == 30 && set == false)
            {
                threshold = DetectThreshold(accValues); // Get threshold. Any threshold less than 9 is not acceptable as g = 9.8m/s2
                if (threshold > 9)
                {
                    set = true;
                }

                Log.i("Setting threshold = ", "" + threshold);
            }

            accValues[currentPointer] = acc_mod - threshold;
            EditText stepText = this.findViewById(R.id.editText);
            stepText.setText("steps = " + stepCount);

           // Log.i("currentPointer val = ", "" + currentPointer);
            if (currentPointer >= window_size)
            {
                if (set == true)
                {
                    prev_acc = acc_mod;
                }
                double smooth_val = averageFloat(currentPointer - window_size, accValues, currentPointer); // Smoothen the signal
                smooth_acc = smooth_val;

            }
            else
            {
                smooth_acc = acc_mod;
            }

            currentPointer++;
            if (currentPointer == 100) {
                currentPointer = 0;
            }
        }
    }

    public double DetectThreshold(double[] values)
    {
        double diff = 0;
        Map<Double, Integer> modeDictionary = new HashMap<Double, Integer>();

        for (int i = 1; i < values.length; i++)
        {
            diff = Math.abs(values[i] - values[i - 1]);
            if (diff < 0.2)
            {
                boolean updated = false;
                for (Double value : modeDictionary.keySet())
                {
                    if (Math.abs(values[i] - value)< 0.2 * value)
                    {
                        updated = true;
                        modeDictionary.put(value, modeDictionary.get(value) + 1);
                        break;
                    }
                }

                if (updated == false)
                {
                    modeDictionary.put(values[i], 1);
                }
            }
        }

        double highestFreq = 0;
        double threshold = 0;
        for (Double value: modeDictionary.keySet())
        {
            if (modeDictionary.get(value) >= highestFreq)
            {
                threshold = value;
            }
        }

        return threshold;
    }

    public boolean DetectPeak(double value)
    {
        if (prev_acc > threshold + 2.5)
        {
            Log.i("Ignored noise:", "value : " + prev_acc);
            return false; // noise
        }

        if (value < prev_acc && prev_acc > threshold + 0.5)
        {
            return true;
        }

        return false;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

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

    private double[] movingAverages(double[] values) // Given a vector value, this finds moving averages
    {
        double[] newValues = new double[values.length];
        int i =0;

        for(i =0; i<values.length - window_size; i++)
        {
            double sum = 0;
            for (int j = i; j < i + window_size; j++)
            {
                sum += values[j];
            }

            newValues[i] = sum/window_size;
        }

        for (;i<values.length; i++)
        {
            newValues[i] = values[i];
        }

        return newValues;
    }

    private double vectorMod(double[] vec1) // Function to find modulus of a vector
    {
        double val = vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2];
        val = Math.sqrt(val);
        return (val);
    }
}
