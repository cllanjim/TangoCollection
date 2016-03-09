/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.experiments.javamotiontracking;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.projecttango.tangoutils.TangoPoseUtilities;

import org.rajawali3d.surface.IRajawaliSurface;
import org.rajawali3d.surface.RajawaliSurfaceView;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbInterface;

import com.androidplot.xy.SimpleXYSeries;

import com.androidplot.xy.*;
import java.util.Arrays;


import java.nio.ByteBuffer;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Random;

import android.os.Handler;
/**
 * Main Activity class for the Motion Tracking API Sample. Handles the connection to the Tango
 * service and propagation of Tango pose data to OpenGL and Layout views. OpenGL rendering logic is
 * delegated to the {@link MotionTrackingRajawaliRenderer} class.
 */
public class MotionTrackingActivity extends Activity implements View.OnClickListener {

    private static final String TAG = MotionTrackingActivity.class.getSimpleName();
    private static final int SECS_TO_MILLISECS = 1000;
    private static final double UPDATE_INTERVAL_MS = 100.0f;
    private static final DecimalFormat FORMAT_THREE_DECIMAL = new DecimalFormat("0.000");

    private Tango mTango;
    private TangoConfig mConfig;
    private TextView mDeltaTextView;
    private TextView mPoseCountTextView;
    private TextView mPoseTextView;
    private TextView mQuatTextView;
    private TextView mPoseStatusTextView;
    private TextView mTangoServiceVersionTextView;
    private TextView mApplicationVersionTextView;
    private TextView mTangoEventTextView;
    private Button mMotionResetButton;
    private boolean mIsAutoRecovery;
    private MotionTrackingRajawaliRenderer mRenderer;

    // State tracking variables for pose callback handler.  If these are
    // referenced from the main thread, they need to be synchronized with the
    // pose handler callback.
    private double mPreviousTimeStamp = 0.0;
    private int mPreviousPoseStatus = TangoPoseData.POSE_INVALID;
    private int mCount = 0;
    private double mTimeToNextUpdate = UPDATE_INTERVAL_MS;


    // USB variables
    UsbDevice device;
    UsbManager manager;
    UsbInterface devInterface;
    UsbEndpoint epIn;
    UsbEndpoint epOut;
    UsbDeviceConnection devConnection;

    // Variables for camera
    UsbDevice deviceCamera;
    UsbManager managerCamera;

    UsbInterface devInterfaceCamera;
    UsbEndpoint epInCamera;
    UsbEndpoint epOutCamera;
    UsbDeviceConnection devConnectionCamera;
    private static final String ACTION_USB_PERMISSION = "com.mobilemerit.usbhost.USB_PERMISSION";
    PendingIntent mPermissionIntent;
    PendingIntent mPermissionIntentCamera;


    // redraws a plot whenever an update is received:
    SimpleXYSeries series1 = null;
    private XYPlot plot;



    final byte[] getSpec = {
            (byte) 0xC1, (byte) 0xC0, //start bytes
            0x00, 0x10, // protocol version
            0x00, 0x00, // flags
            0x00, 0x00, // error number
            0x00, 0x10, 0x10, 0x00, // message type
            0x00, 0x00, 0x00, 0x00, // regarding (user-specified)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved
            0x00, // checksum type
            0x00, // immediate length
            0x00, 0x00, 0x00, 0x00, // unused (immediate data)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // unused (remainder of immediate data)
            0x14, 0x00, 0x00, 0x00,// bytes remaining
            // optional payload
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // checksum
            (byte) 0xC5, (byte) 0xC4, (byte) 0xC3, (byte) 0xC2 // footer
    };

    private DataOutputStream tangoPoseOutput;
    private DataOutputStream tangoDepthOutput;
    private DataOutputStream tangoCamOutput;
    private DataOutputStream tangoSpectroOutput;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_motion_tracking);
        Intent intent = getIntent();
        mIsAutoRecovery = intent.getBooleanExtra(StartActivity.KEY_MOTIONTRACKING_AUTORECOVER,
                false);
        // Instantiate the Tango service
        mTango = new Tango(this);
        mConfig = setupTangoConfig(mTango, mIsAutoRecovery);
        setupTextViewsAndButtons(mConfig);
        mRenderer = setupGLViewAndRenderer();

        plot = (XYPlot) findViewById(R.id.plot);
        series1 = new SimpleXYSeries("series1");
        series1.useImplicitXVals();
        plot.addSeries(series1, new LineAndPointFormatter(Color.BLUE, Color.RED, Color.BLACK, null));

        File root = new File(Environment.getExternalStorageDirectory()+File.separator+"CraigData");
        File poutput = new File(root, "pose.dat");
        File doutput = new File(root, "depth.dat");
        File coutput = new File(root, "cam.dat");
        File soutput = new File(root, "spec.dat");

        FileOutputStream pfos;
        FileOutputStream dfos;
        FileOutputStream cfos;
        FileOutputStream sfos;
        try {
            pfos = new FileOutputStream(poutput);
            tangoPoseOutput = new DataOutputStream(pfos);

            cfos = new FileOutputStream(coutput);
            tangoCamOutput = new DataOutputStream(cfos);

            dfos = new FileOutputStream(doutput);
            tangoDepthOutput = new DataOutputStream(dfos);

            sfos = new FileOutputStream(soutput);
            tangoSpectroOutput = new DataOutputStream(sfos);

        } catch (IOException e) {
            e.printStackTrace();
        }


        setupSpectrometer();


        setupCameraCapture();





    }

    private void setupSpectrometer() {
        manager = (UsbManager) getSystemService(Context.USB_SERVICE);
		/*
		 * this block required if you need to communicate to USB devices it's
		 * take permission to device
		 * if you want than you can set this to which device you want to communicate
		 */
        // ------------------------------------------------------------------
        mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(
                ACTION_USB_PERMISSION), 0);
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        registerReceiver(mUsbReceiver, filter);
        // -------------------------------------------------------------------
        HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
        Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();
        while (deviceIterator.hasNext()) {
            device = deviceIterator.next();
            manager.requestPermission(device, mPermissionIntent);
            Log.d("Device", " " + device.getDeviceId());
            if (device.getProductId() == 0x4200) {
                devInterface = device.getInterface(0);
                // https://github.com/uvwxy/android-ambit/blob/57bf1f4642b01680b26c6c807283f93dd8386130/src/de/uvwxy/android/ambit/lib/AndroidAmbitDevice.java

                UsbEndpoint ep0 = devInterface.getEndpoint(0);
                UsbEndpoint ep1 = devInterface.getEndpoint(1);
                if (ep0.getDirection() == UsbConstants.USB_DIR_IN) {
                    epIn = ep0;
                    epOut = ep1;

                } else {
                    epIn = ep1;
                    epOut = ep0;

                }
                if (manager.hasPermission(device)) {
                    devConnection = manager.openDevice(device);
                    // Reset
                    // Wait one second
                    // Set integration time (10,000 uSec = 10ms)
                    byte[] intTime = {
                            (byte) 0xC1, (byte) 0xC0, //start bytes
                            0x00, 0x10, // protocol version
                            0x00, 0x00, // flags
                            0x00, 0x00, // error number
                            0x10, 0x00, 0x11, 0x00, // message type
                            0x00, 0x00, 0x00, 0x00, // regarding (user-specified)
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved
                            0x00, // checksum type
                            0x04, // immediate length
                            0x10, 0x27, 0x00, 0x00, // integration time (immediate data)
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // unused (remainder of immediate data)
                            0x14, 0x00, 0x00, 0x00 ,// bytes remaining
                            // optional payload
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // checksum
                            (byte) 0xC5, (byte) 0xC4,(byte)  0xC3,(byte)  0xC2 // footer
                    };
                    Log.d("USB", "bytes transfered intTime " + devConnection.bulkTransfer(epOut, intTime, 64, 1000));

                    // Set binning factor
                    byte[] binFact = {
                            (byte) 0xC1, (byte) 0xC0, //start bytes
                            0x00, 0x10, // protocol version
                            0x00, 0x00, // flags
                            0x00, 0x00, // error number
                            (byte) 0x90, 0x02, 0x11, 0x00, // message type
                            0x00, 0x00, 0x00, 0x00, // regarding (user-specified)
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved
                            0x00, // checksum type
                            0x01, // immediate length
                            0x00, 0x00, 0x00, 0x00, // binning mode (0) was too lazy to move the next three bytes to next line (immediate data)
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // unused (remainder of immediate data)
                            0x14, 0x00, 0x00, 0x00 ,// bytes remaining
                            // optional payload
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // checksum
                            (byte) 0xC5, (byte) 0xC4,(byte)  0xC3,(byte)  0xC2 // footer
                    };
                    Log.d("USB", "bytes transfered binning factor " + devConnection.bulkTransfer(epOut, binFact, 64, 1000));
                    final Handler specHandler = new Handler();
                    final int specDelay = 20; //milliseconds

                    final Random rand = new Random();
                    specHandler.postDelayed(new Runnable() {
                        public void run() {
                            // update spectrum UI occasionally
                            captureSpectrum(rand.nextInt(10) == 0);
                            specHandler.postDelayed(this, specDelay);
                        }
                    }, specDelay);
                    break;
                }
            }
        }

    }
    private void setupCameraCapture() {
        managerCamera = (UsbManager) getSystemService(Context.USB_SERVICE);
		/*
		 * this block required if you need to communicate to USB devices it's
		 * take permission to device
		 * if you want than you can set this to which device you want to communicate
		 */
        // ------------------------------------------------------------------
        mPermissionIntentCamera = PendingIntent.getBroadcast(this, 0, new Intent(
                ACTION_USB_PERMISSION), 0);
        IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
        registerReceiver(mUsbReceiver, filter);
        // -------------------------------------------------------------------
        HashMap<String, UsbDevice> deviceList = managerCamera.getDeviceList();
        Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();
        while (deviceIterator.hasNext()) {
            deviceCamera = deviceIterator.next();
            managerCamera.requestPermission(deviceCamera, mPermissionIntentCamera);
            if ((deviceCamera.getProductId() == 221) && (deviceCamera.getVendorId()==1240)) {
                Log.d("Device", "found 2221");
                // Based on Microchip's MCP2221.java
                for (int j = 0; j < deviceCamera.getInterfaceCount(); j++){
                    if (deviceCamera.getInterface(j).getInterfaceClass() == UsbConstants.USB_CLASS_HID){
                        Log.d("Device","got interface");
                        devInterfaceCamera = deviceCamera.getInterface(j);
                        for (int k = 0; k < devInterfaceCamera.getEndpointCount(); k++) {
                            if (devInterfaceCamera.getEndpoint(k).getDirection() == UsbConstants.USB_DIR_OUT) {
                                Log.d("Device","got out endpoint");
                                epOutCamera = devInterfaceCamera.getEndpoint(k);
                            } else {
                                epInCamera = devInterfaceCamera.getEndpoint(k);
                            }
                        }

                    }
                }

                if (managerCamera.hasPermission(deviceCamera)) {
                    devConnectionCamera = manager.openDevice(deviceCamera);

                    // set GP0 to GPIO
//                    byte[] setGPIO = {
//                            (byte) 0xB1, // write flash data
//                            0x01, // write GP settings
//                            (byte) 0x00, // GP0 power up settings (output, initial value 1)
//                            (byte) 0x00, // GP1 power up settings (output, initial value 0)
//                            (byte) 0x00, // GP2 power up settings (ignore)
//                            (byte) 0x00, // GP3 power up settings (ignore)
//                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 6-15 we don't care
//                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 16-25 we don't care
//                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 26-35 we don't care
//                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 36-45 we don't care
//                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 46-55 we don't care
//                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00// 8 bytes- Bytes 56-63 we don't care
//                    };
//                    Log.d("bytes transfered: "," " + devConnectionCamera.bulkTransfer(epOutCamera, setGPIO, 64, 1000));
//                    byte[] resp = new byte[64];
//                    devConnectionCamera.bulkTransfer(epInCamera, resp, 64, 1000);
//                    Log.d("camera setup", "should be 0 " + resp[1]);

                    // read the GP values
                    byte[] readGP = new byte[64];
                    readGP[0] = (byte) 0xB0;
                    readGP[1] = 0x01;
                    byte []resp = new byte[64];
                    Log.d("bytes transfered: "," " + devConnectionCamera.bulkTransfer(epOutCamera, readGP, 64, 1000));
                    devConnectionCamera.bulkTransfer(epInCamera, resp, 64, 1000);
                    Log.d("camera setup", "GP0 power up " + resp[4] +" GP1 power up " + resp[5] +" GP2 power up " + resp[6] +" GP3 power up " + resp[7]  );


//                    final Handler camHandler = new Handler();
//                    final int camDelay = 350; //milliseconds
//                    camHandler.postDelayed(new Runnable() {
//                        public void run() {
//                            startCapture();
//                            android.os.SystemClock.sleep(50);
//                            endCapture();
//                            camHandler.postDelayed(this, camDelay);
//                        }
//                    }, camDelay);

                    break;
                }
            }
        }

    }
    private void captureSpectrum(boolean updateUI){
        devConnection.bulkTransfer(epOut, getSpec, 64, 1000);
        byte[] spec = new byte[2112];
        //Log.d("USB", "bytes recieved " + devConnection.bulkTransfer(epIn, spec, 2112, 1000));
        devConnection.bulkTransfer(epIn, spec, 2112, 1000);
        // Attempt to plot them
        try {
            tangoSpectroOutput.writeLong(System.currentTimeMillis());
            tangoSpectroOutput.write(spec);
        } catch (IOException e) {
        e.printStackTrace();
    }
        Number[] intensities = new Number[1024];
        int numCount = 0;
        for (int i = 44; i <= 2091; i = i + 2) {
            intensities[numCount] = ByteBuffer.wrap(new byte[]{spec[i + 1], spec[i]}).getShort();
            numCount += 1;
        }
        if (updateUI) {
            Log.d("Plot", "plotted?");

            series1.setModel(Arrays.asList(intensities), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY);

            plot.redraw();
        }
    }

    private void startCapture(){
        Log.d("USB", "starting capture");

        byte[] setGPIO = {
                0x50, // set GPIO output values
                (byte)0xFF, // don't care
                0x01, // Modify GP0 output
                0x00, // Set GP0 to be 0
                0x00, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                // GP1
                0x00, // Don't modify output
                0x00, // Set GP1 to be 0
                0x00, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                // GP2
                0x00, // Don't modify output
                0x00, // Set GP0 to be 0 (shouldn't matter)
                0x00, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                // GP3
                0x00, // Don't modify output
                0x00, // Set GP0 to be 0 (shouldn't matter)
                0x00, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 18-27 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 28-37 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 38-47 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 48-57 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00// 6 bytes- Bytes 58-63 reserved
        };

        Log.d("USB", "start cap: bytes transfered intTime " + devConnectionCamera.bulkTransfer(epOutCamera, setGPIO, 64, 1000));
        byte[] resp = new byte[64];
        devConnectionCamera.bulkTransfer(epInCamera, resp, 64, 1000);
        Log.d("USB", "RESP GPIO values 0 0 1 0? " + resp[2] + " " + resp[3] + " " + resp[4] + " " + resp[5]);

        // capture time is System.currentTimeMillis() + some offset which we can calculate later?

        try {
            tangoCamOutput.writeLong(System.currentTimeMillis());
            Log.d("cam", "photo taken");
        } catch (IOException e) {
            e.printStackTrace();
        }
        byte[] getGPIOReq = new byte[64];
        getGPIOReq[0] = 0x51;
        Log.d("USB", "start cap: bytes transfered intTime " + devConnectionCamera.bulkTransfer(epOutCamera, getGPIOReq, 64, 1000));

        byte[] getGPIO = new byte[64];
        devConnectionCamera.bulkTransfer(epInCamera, getGPIO, 64, 1000);
        Log.d("USB", "GPIO values 0 0 1 0? " + getGPIO[2] + " " + getGPIO[3] + " " + getGPIO[4] + " " + getGPIO[5]);
    }
    private void endCapture(){
        Log.d("USB", "ending capture");
        byte[] setGPIO = {
                0x50, // set GPIO output values
                0x00, // don't care
                0x01, // Modify GP0 output
                0x01, // Set GP0 to be 1
                0x00, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                // GP1
                0x00, // Don't modify output
                0x00, // Set GP1 to be 0
                0x01, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                // GP2
                0x00, // Don't modify output
                0x00, // Set GP0 to be 0 (shouldn't matter)
                0x00, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                // GP3
                0x00, // Don't modify output
                0x00, // Set GP0 to be 0 (shouldn't matter)
                0x00, // Don't change GPIO direction
                0x00, // New GPIO pin direction (but should be ignored since previous bit is 0)
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 18-27 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 28-37 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 38-47 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 10 bytes- Bytes 48-57 reserved
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00// 6 bytes- Bytes 58-63 reserved
        };
        Log.d("USB", "end cap: bytes transfered intTime " + devConnectionCamera.bulkTransfer(epOutCamera, setGPIO, 64, 1000));
        byte[] getGPIOReq = new byte[64];
        getGPIOReq[0] = 0x51;
        Log.d("USB", "start cap: bytes transfered intTime " + devConnectionCamera.bulkTransfer(epOutCamera, getGPIOReq, 64, 1000));

        byte[] getGPIO = new byte[64];
        devConnectionCamera.bulkTransfer(epInCamera, getGPIO, 64, 1000);
        Log.d("USB", "GPIO values 0 1 1 0? " + getGPIO[2] + " " + getGPIO[3] + " " + getGPIO[4] + " " + getGPIO[5]);
    }

    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {

        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    UsbDevice device = (UsbDevice) intent
                            .getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    if (intent.getBooleanExtra(
                            UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        if (device != null) {
                            // call method to set up device communication
                        }
                    } else {
                        Log.d("ERROR", "permission denied for device " + device);
                    }
                }
            }
        }
    };
    /**
     * Sets Rajawalisurface view and its renderer. This is ideally called only once in onCreate.
     */
    private MotionTrackingRajawaliRenderer setupGLViewAndRenderer(){

        // Configure OpenGL renderer
        MotionTrackingRajawaliRenderer renderer = new MotionTrackingRajawaliRenderer(this);
        // OpenGL view where all of the graphics are drawn
        RajawaliSurfaceView glView = (RajawaliSurfaceView) findViewById(R.id.gl_surface_view);
        glView.setEGLContextClientVersion(2);
        glView.setRenderMode(IRajawaliSurface.RENDERMODE_CONTINUOUSLY);
        glView.setSurfaceRenderer(renderer);
        return renderer;

    }

    /**
     * Sets Texts views to display statistics of Poses being received. This also sets the buttons
     * used in the UI. Please note that this needs to be called after TangoService and Config
     * objects are initialized since we use them for the SDK related stuff like version number
     * etc.
     */
    private void setupTextViewsAndButtons(TangoConfig config){
        // Text views for displaying translation and rotation data
        mPoseTextView = (TextView) findViewById(R.id.pose);
        mQuatTextView = (TextView) findViewById(R.id.quat);
        mPoseCountTextView = (TextView) findViewById(R.id.posecount);
        mDeltaTextView = (TextView) findViewById(R.id.deltatime);
        mTangoEventTextView = (TextView) findViewById(R.id.tangoevent);

        // Text views for the status of the pose data and Tango library versions
        mPoseStatusTextView = (TextView) findViewById(R.id.status);
        mTangoServiceVersionTextView = (TextView) findViewById(R.id.version);
        mApplicationVersionTextView = (TextView) findViewById(R.id.appversion);

        // Buttons for selecting camera view and Set up button click listeners
        findViewById(R.id.first_person_button).setOnClickListener(this);
        findViewById(R.id.third_person_button).setOnClickListener(this);
        findViewById(R.id.top_down_button).setOnClickListener(this);

        // Button to reset motion tracking
        mMotionResetButton = (Button) findViewById(R.id.resetmotion);
        // Set up button click listeners
        mMotionResetButton.setOnClickListener(this);

        // Display the library version for debug purposes
        mTangoServiceVersionTextView.setText(config.getString("tango_service_library_version"));
        PackageInfo packageInfo;
        try {
            packageInfo = this.getPackageManager().getPackageInfo(this.getPackageName(), 0);
            mApplicationVersionTextView.setText(packageInfo.versionName);
        } catch (NameNotFoundException e) {
            e.printStackTrace();
        }
    }

    /**
     * Sets up the tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(Tango tango, boolean isAutoRecovery){
        // Create a new Tango Configuration and enable the MotionTrackingActivity API
        TangoConfig config = new TangoConfig();
        config = tango.getConfig(config.CONFIG_TYPE_CURRENT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);

        // The Auto-Recovery ToggleButton sets a boolean variable to determine
        // if the
        // Tango service should automatically attempt to recover when
        // / MotionTrackingActivity enters an invalid state.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, isAutoRecovery);
        Log.i(TAG, "Auto Reset: " + mIsAutoRecovery);
        return  config;
    }

    /**
     * Set up the callback listeners for the Tango service, then begin using the Motion
     * Tracking API. This is called in response to the user clicking the 'Start' Button.
     */
    private void setTangoListeners() {
        // Lock configuration and connect to Tango
        // Select coordinate frame pair
        final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        // Listen for new Tango data
        mTango.connectListener(framePairs, new OnTangoUpdateListener() {
            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                // Update the OpenGL renderable objects with the new Tango Pose
                // data.  Note that locking for thread safe access with the
                // OpenGL loop is done entirely in the renderer.
                mRenderer.updateDevicePose(pose);

                // Now lets log some interesting statistics of Motion Tracking
                // like Delta Time between two Poses, number of poses since the
                // initialization state.
                final double deltaTime = (pose.timestamp - mPreviousTimeStamp)
                    * SECS_TO_MILLISECS;

                mPreviousTimeStamp = pose.timestamp;
                // Log whenever Motion Tracking enters an invalid state
                if (!mIsAutoRecovery && (pose.statusCode == TangoPoseData.POSE_INVALID)) {
                    Log.w(TAG, "Invalid State");
                }
                if (mPreviousPoseStatus != pose.statusCode) {
                    mCount = 0;
                }
                mCount++;
                final int count = mCount;
                mPreviousPoseStatus = pose.statusCode;


                // Throttle updates to the UI based on UPDATE_INTERVAL_MS.
                mTimeToNextUpdate -= deltaTime;
                boolean updateUI = false;
                if(mTimeToNextUpdate < 0.0) {
                    mTimeToNextUpdate = UPDATE_INTERVAL_MS;
                    updateUI = true;
                }

                // If the pose is not valid, we may not get another callback so make sure to update
                // the UI during this call
                if (pose.statusCode != TangoPoseData.POSE_VALID) {
                    updateUI = true;
                }

                // save values
                if (pose != null) {
                    try {
                        tangoPoseOutput.writeDouble(pose.timestamp);
                        tangoPoseOutput.writeDouble(pose.translation[0]);
                        tangoPoseOutput.writeDouble(pose.translation[1]);
                        tangoPoseOutput.writeDouble(pose.translation[2]);

                        tangoPoseOutput.writeDouble(pose.rotation[0]);
                        tangoPoseOutput.writeDouble(pose.rotation[1]);
                        tangoPoseOutput.writeDouble(pose.rotation[2]);
                        tangoPoseOutput.writeDouble(pose.rotation[3]);

                    } catch (IOException e) {
                        e.printStackTrace();
                    }

                    if (updateUI) {

                        final String translationString =
                                TangoPoseUtilities.getTranslationString(pose, FORMAT_THREE_DECIMAL);
                        final String quaternionString =
                                TangoPoseUtilities.getQuaternionString(pose, FORMAT_THREE_DECIMAL);
                        final String status = TangoPoseUtilities.getStatusString(pose);

                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                // Display pose data on screen in TextViews.
                                mPoseTextView.setText(translationString);
                                mQuatTextView.setText(quaternionString);
                                mPoseCountTextView.setText(Integer.toString(count));
                                mDeltaTextView.setText(FORMAT_THREE_DECIMAL.format(deltaTime));
                                mPoseStatusTextView.setText(status);
                            }
                        });
                    }
                }
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData arg0) {
                // We are not using TangoXyzIjData for this application
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mTangoEventTextView.setText(event.eventKey + ": " + event.eventValue);
                    }
                });
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // We are not using onFrameAvailable for this application
            }
        });
    }

    /**
     * Reset motion tracking to last known valid pose. Once this function is called,
     * Motion Tracking restarts as such we may get initializing poses again. Developer should make
     * sure that user gets enough feed back in that case.
     */
    private void motionReset() {
        mTango.resetMotionTracking();
    }

    @Override
    protected void onPause() {
        super.onPause();
        try {
            mTango.disconnect();
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        try {
            setTangoListeners();
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT).show();
        } catch (SecurityException e) {
            Toast.makeText(getApplicationContext(), R.string.motiontrackingpermission,
                    Toast.LENGTH_SHORT).show();
        }
        try {
            mTango.connect(mConfig);
        } catch (TangoOutOfDateException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoOutOfDateException,
                    Toast.LENGTH_SHORT).show();
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError, Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.first_person_button:
                mRenderer.setFirstPersonView();
                break;
            case R.id.top_down_button:
                mRenderer.setTopDownView();
                break;
            case R.id.third_person_button:
                mRenderer.setThirdPersonView();
                break;
            case R.id.resetmotion:
                motionReset();
                break;
            default:
                Log.w(TAG, "Unknown button click");
                return;
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mRenderer.onTouchEvent(event);
        return true;
    }
    @Override
    public void onStop() {
        super.onStop();
        devConnection.close();
        devConnectionCamera.close();
    }
}