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
import com.microchip.android.mcp2221comm.Mcp2221Config;

import com.microchip.android.mcp2221comm.Mcp2221Comm;
import com.microchip.android.mcp2221comm.Mcp2221Constants;
import com.microchip.android.microchipusb.Constants;
import com.microchip.android.microchipusb.MCP2221;
import com.microchip.android.microchipusb.MicrochipUsb;

import android.widget.Toast;
import android.view.Gravity;


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
    PendingIntent mPermissionIntent;
    PendingIntent mPermissionIntentCamera;


    // redraws a plot whenever an update is received:
    SimpleXYSeries series1 = null;
    private XYPlot plot;

    /** Microchip Product ID. */
    protected static final int MCP2221_PID = 0xDD;
    /** Microchip Vendor ID. */
    protected static final int MCP2221_VID = 0x4D8;
    /** TAG to be used for logcat. */
   // protected static final String TAG = "MainActivity";
    /** Custom toast - displayed in the center of the screen. */
    private static Toast sToast;
    /** USB permission action for the USB broadcast receiver. */
    private static final String ACTION_USB_PERMISSION = "com.microchip.android.USB_PERMISSION";
    /** public member to be used in the test project. */
    public MCP2221 mcp2221;
    /** public member to be used in the test project. */
    public Mcp2221Comm mcp2221Comm;

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

        File root = new File(Environment.getExternalStorageDirectory()+File.separator+"CraigData"+System.currentTimeMillis());
        try{
            if(root.mkdir()) {
                System.out.println("Directory created");
            } else {
                System.out.println("Directory is not created");
            }
        }catch(Exception e){
            e.printStackTrace();
        }
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


        //setupCameraCapture();
        sToast = Toast.makeText(this, "", Toast.LENGTH_SHORT);
        sToast.setGravity(Gravity.CENTER, 0, 0);

        mPermissionIntent =
                PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);

        final IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);

        registerReceiver(mUsbReceiver, filter);

        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        registerReceiver(mUsbReceiver, filter);

        Constants result = null;

        mcp2221 = new MCP2221(this);
        result = MicrochipUsb.getConnectedDevice(this);
        if (result == Constants.MCP2221) {
            // try to open a connection
            result = mcp2221.open();

            switch (result) {
                case SUCCESS:
                    sToast.setText("MCP2221 connected");
                    sToast.show();
                    mcp2221Comm = new Mcp2221Comm(mcp2221);
                    setupMCP2221();
                    break;
                case CONNECTION_FAILED:
                    sToast.setText("Connection failed");
                    sToast.show();
                    break;
                case NO_USB_PERMISSION:
                    mcp2221.requestUsbPermission(mPermissionIntent);
                    break;
                default:
                    break;
            }
        }

        final Handler camHandler = new Handler();
        final byte b0 = 0;
        final byte b1 = 1;
        final int camDelay = 350; //milliseconds
        camHandler.postDelayed(new Runnable() {
            public void run() {
                mcp2221Comm.setGpPinValue(b1, b0); // make sure focus is connected to ground
                mcp2221Comm.setGpPinValue(b0, b0); // trigger the shutter
                //startCapture();
                try {
                    tangoCamOutput.writeLong(System.currentTimeMillis());
                    Log.d("cam", "photo taken");
                } catch (IOException e) {
                    e.printStackTrace();
                }
                android.os.SystemClock.sleep(50); // wait a bit
                mcp2221Comm.setGpPinValue(b0,b1); // bring the shutter back high

                camHandler.postDelayed(this, camDelay);
            }
        }, camDelay);





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
                Log.d("INTERFACE CLASS", "class: " + devInterface.getInterfaceClass());
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
        @Override
        public void onReceive(final Context context, final Intent intent) {
            final String action = intent.getAction();
            Log.d("USB", "onReceive");
            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    final UsbDevice device =
                            (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);

                    // is usb permission has been granted, try to open a connection
                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        if (device != null) {
                            // call method to set up device communication
                            final Constants result = mcp2221.open();

                            if (result != Constants.SUCCESS) {
                                sToast.setText("Could not open MCP2221 connection");
                                sToast.show();
                            } else {
                                mcp2221Comm = new Mcp2221Comm(mcp2221);
                                sToast.setText("MCP2221 connection opened");
                                sToast.show();
                                setupMCP2221();

                            }
                        }
                    } else {
                        sToast.setText("USB Permission Denied");
                        sToast.show();
                    }
                }
            }

            if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
                sToast.setText("Device Detached");
                sToast.show();
                // close the connection and
                // release all resources
                mcp2221.close();
                // leave a bit of time for the COM thread to close
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    // e.printStackTrace();
                }
                mcp2221Comm = null;
            }

            if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
                sToast.setText("Device Attached");
                sToast.show();
                final UsbDevice device =
                        (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                if (device != null) {

                    // only try to connect if an MCP2221 is attached
                    if (device.getVendorId() == MCP2221_VID && device.getProductId() == MCP2221_PID) {
                        final Constants result = mcp2221.open();

                        switch (result) {
                            case SUCCESS:
                                sToast.setText("MCP2221 Connection Opened");
                                sToast.show();
                                Log.d("USB", "Connection opened");
                                mcp2221Comm = new Mcp2221Comm(mcp2221);
                                setupMCP2221();

                                // if the nav drawer isn't open change the menu icon to show the MCP
                                // is connected

                                break;
                            case CONNECTION_FAILED:
                                sToast.setText("Connection Failed");
                                sToast.show();
                                Log.d("USB", "Connection failed");

                                break;
                            case NO_USB_PERMISSION:
                                Log.d("USB", "no permission");

                                mcp2221.requestUsbPermission(mPermissionIntent);
                                break;
                            default:
                                break;
                        }
                    }
                }
            }

        }
    };
    void setupMCP2221(){
        Mcp2221Config conf = new Mcp2221Config();
        byte[] pinDesDir = {0,0,0,0};
        conf.setGpPinDesignations(pinDesDir);
        conf.setGpPinDirections(pinDesDir);
        byte[] pinVals = {1,0,0,0};
        conf.setGpPinValues(pinVals);
        mcp2221Comm.setSRamSettings(conf, false, false, false, false, false, false, true);
        Log.d("MCP2221", "values set");


    }
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
        // close the connection and release all resources
        if(mUsbReceiver != null) {
            unregisterReceiver(mUsbReceiver);
        }
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
