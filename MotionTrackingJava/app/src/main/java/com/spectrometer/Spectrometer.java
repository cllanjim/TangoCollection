/**
 * Created by craig on 3/14/16.
 * (very) heavily based off of Microchip's MCP2221 code
 */
package com.spectrometer;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.Context;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.hardware.usb.UsbRequest;
import android.os.Handler;

import com.microchip.android.microchipusb.Constants; // should really find a way to replace this

import java.nio.ByteBuffer;
import java.util.HashMap;


public class Spectrometer {
    /** Spectrometer Product ID. */
    private static final int SPEC_PID = 0x4200;
    /** Spectrometer Vendor ID. */
    private static final int SPEC_VID = 0x2457;
    /** USB HID packet size for the MCP2221. */
    private static final int HID_PACKET_SIZE = 64;

    /** USB connection for the spectrometer. */
    private UsbDeviceConnection mSpecConnection;
    /** Spectrometer USB device. */
    private UsbDevice mSpecDevice;
    /** USB HID OUT endpoint. Used for sending commands to the Spectrometer via the HID interface. */
    private UsbEndpoint mSpecEpOut;
    /** USB HID IN endpoint. Used for getting data from the Spectrometer via the HID interface. */
    private UsbEndpoint mSpecEpIn;
    /** Spectrometer HID interface reference. */
    private UsbInterface mSpecInterface;
    /** USB request used for queuing data to the OUT USB endpoint. */
    private final UsbRequest mSpecUsbOutRequest = new UsbRequest();
    /** USB request used for getting data from the IN USB endpoint queue. */
    private final UsbRequest mSpecUsbInRequest = new UsbRequest();
    /** UsbManager used to scan for connected Spectrometer devices and grant USB permission. */
    private final UsbManager mUsbManager;

    protected static final String TAG = "Spectrometer";

    /**
     * Create a new MCP2221.
     *
     * @param receivedActivity
     *            (Activity)<br>
     *            A reference to the activity from which this constructor is called.
     */
    public Spectrometer(final Activity receivedActivity) {
        super();
        mUsbManager = (UsbManager) receivedActivity.getSystemService(Context.USB_SERVICE);

    }


    /**
     * Close the communication with the MCP2221, release the USB interface <br>
     * and all resources related to the object.
     */
    public final void close() {
        if (mSpecConnection != null) {
            mSpecConnection.releaseInterface(mSpecInterface);
            mSpecConnection.close();
            mSpecConnection = null;
        }
    }

    /**
     * Open a connection to the MCP2221.
     *
     * @return (Constants) Constants.SUCCESS if the connection was established
     *         Constants.ERROR_MESSAGE if the connection failed
     */
    public final Constants open() {

        HashMap<String, UsbDevice> deviceList;
        deviceList = mUsbManager.getDeviceList();

        UsbInterface tempInterface;

        for (String key : deviceList.keySet()) {
            mSpecDevice = deviceList.get(key);

            if (mSpecDevice.getVendorId() == SPEC_VID
                    && mSpecDevice.getProductId() == SPEC_PID) {
                // we found the spectrometer
                // Now go through the interfaces until we find the vendor specified one, should be the 0th
                for (int i = 0; i < mSpecDevice.getInterfaceCount(); i++) {
                    tempInterface = mSpecDevice.getInterface(i);
                    if (tempInterface.getInterfaceClass() == UsbConstants.USB_CLASS_VENDOR_SPEC) {
                        // we found the interface
                        mSpecInterface = tempInterface;
                        for (int j = 0; j < mSpecInterface.getEndpointCount(); j++) {
                            if (mSpecInterface.getEndpoint(j).getDirection() == UsbConstants.USB_DIR_OUT) {
                                // found the OUT USB endpoint
                                mSpecEpOut = mSpecInterface.getEndpoint(j);
                            } else {
                                // found the IN USB endpoint
                                mSpecEpIn = mSpecInterface.getEndpoint(j);
                            }
                        }
                        break;
                    }
                }
                // if the user granted USB permission
                // try to open a connection
                if (mUsbManager.hasPermission(mSpecDevice)) {
                    mSpecConnection = mUsbManager.openDevice(mSpecDevice);
                } else {
                    return Constants.NO_USB_PERMISSION;
                }
                break;
            }
        }
        if (mSpecConnection == null) {
            return Constants.CONNECTION_FAILED;
        } else {
            return Constants.SUCCESS;
        }
    }

    /**
     * Request temporary USB permission for the connected Spec. <br>
     * Success or failure is returned via the PendingIntent permissionIntent.
     *
     * @param permissionIntent
     *            (PendingIntent) <br>
     *
     */
    public final void requestUsbPermission(final PendingIntent permissionIntent) {
        mUsbManager.requestPermission(mSpecDevice, permissionIntent);
    }

    /**
     * Sends an HID command to the Spec and retrieves the reply.
     *
     * @param data
     *            (ByteBuffer) 64 bytes of data to be sent
     * @return (ByteBuffer) 64 bytes of data received as a response from the Spec <br>
     *         null - if the transaction wasn't successful
     *
     */
    public final ByteBuffer sendData(final ByteBuffer data) {

        if (data.capacity() > HID_PACKET_SIZE) {
            // USB packet size is 64 bytes
            return null;
        }

        ByteBuffer usbCommand = ByteBuffer.allocate(HID_PACKET_SIZE);
        usbCommand = data;

        mSpecUsbOutRequest.initialize(mSpecConnection, mSpecEpOut);
        mSpecUsbInRequest.initialize(mSpecConnection, mSpecEpIn);
        mSpecConnection.claimInterface(mSpecInterface, true);

        // queue the USB command
        mSpecUsbOutRequest.queue(usbCommand, HID_PACKET_SIZE);
        if (mSpecConnection.requestWait() == null) {
            // an error has occurred
            return null;
        }

        ByteBuffer usbResponse = ByteBuffer.allocate(HID_PACKET_SIZE);
        mSpecUsbInRequest.queue(usbResponse, HID_PACKET_SIZE);
        if (mSpecConnection.requestWait() == null) {
            // an error has occurred
            return null;
        }
        return usbResponse;

    }



}
