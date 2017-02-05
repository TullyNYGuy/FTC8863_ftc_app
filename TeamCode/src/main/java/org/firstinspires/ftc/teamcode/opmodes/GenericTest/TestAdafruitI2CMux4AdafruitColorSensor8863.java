package org.firstinspires.ftc.teamcode.opmodes.GenericTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitColorSensor;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitColorSensor8863;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitI2CMux;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.I2cDeviceCommunicator;

import java.util.ArrayList;

/**
 * This opmode demonstates how to use the AdafruitI2CMux class to talk to an I2C mux.
 * It assumes that you have 4 adafruit color sensors connected on these ports of the mux (not the
 * core device interface module):
 * colorSensor1 - port 0
 * colorSensor2 - port 1
 * colorSensor3 - port 2
 * colorSensor4 - port 4
 * The gamepad will be used to switch between the 4 sensors.
 * The phone needs to be configured with an "I2C Device" on the core device interface module port
 * that the mux is connected to. The name of the mux is "mux".
 * It also needs an Adafruit Color Sensor configured on one of the other I2C ports of the core
 * device interface module. It does not matter which port. The name of the color sensor is
 * "colorSensor". The type of sensor is "I2C Device".
 * Lastly, the core device interface module needs to be named "coreDIM" in your phone configuration.
 * <p>
 * The code will create 4 color sensor objects as you might expect. However, once the objects are
 * created, for some reason I don't understand, you always have to communicate with the sensor using
 * the last object created. In this case I create colorSensor1 to 4 in order. Once I create
 * colorSensor2, I can't communicate with color sensor 1 using the colorSensor1 object. I have to
 * use colorSensor2. The selection of which color sensor is communicated with is done only by
 * switching the mux port. Similarly, once I create colorSensor4, I have to use that object to
 * communicate with all 4 color sensors, using the mux port selection to pick which sensor I want
 * to talk to. I'm not sure I understand why. You think you could switch between objects freely.
 * I'm guessing there must be something in the way the hardware map and the objects it creates that
 * causes this issue.
 * <p>
 * IMPORTANT NOTE: this has the potential to lead to bugs. For example, set the mux port to 0 so you
 * are talking to color sensor 1. Set the gain to 64x. Now switch to mux port 1 so you are talking
 * to color sensor 2. Set the gain to 4x. Now switch back to port 0 so you are talking to color
 * sensor 1 again. Get the gain. Since the gain is stored as a property of the object, and the last
 * gain set was 4x, it will tell you the gain is 4x when it was actually set to 64x.
 * THEREFORE: if you are reading a property from the color sensor, it is best to assume it is not
 * valid. I.E. assume the color sensor objects are write only, except for those values that are
 * actually read from the sensor and not from a stored property. For example, color values are
 * actually read from the sensor.
 * <p>
 * WARNING: When you switch from one color sensor on one port to another color sensor on
 * another port, there will be a period of time needed before the data will be valid for the new
 * sensor. This is because it takes time to get data from the new color sensor and populate the
 * data in the data cache. Reports on the FTC forum indicate that it takes from 50 mSec to 150 mSec
 * to get valid data. If you read before then, you will likely get the data from the first color
 * sensor not the new one. In order to avoid this switch the mux port well before you need valid
 * data or put a delay in your code after switching mux ports and before reading color values.
 */
@TeleOp(name = "Test Adafruit I2C Mux 4 AdafruitColorSensor8863", group = "Test")
//@Disabled
public class TestAdafruitI2CMux4AdafruitColorSensor8863 extends LinearOpMode {

    // Put your variable declarations here
    AdafruitI2CMux mux;
    // The name of the mux. This needs to be used when configuring the phone.
    String muxName = "mux";
    // The default address of the I2C mux. If you change that, then you have to change this address.
    byte muxAddress = 0x70;

    I2cDeviceCommunicator colorSensorCommunicator;

    // A color sensor for use in testing the mux. This is my class and is wrapped around ColorSensor.
    // This class integrates both the color sensor and the led control using a discrete output port.
    AdafruitColorSensor8863 colorSensor1;
    AdafruitColorSensor8863 colorSensor2;
    AdafruitColorSensor8863 colorSensor3;
    AdafruitColorSensor8863 colorSensor4;

    // The next variable will point to last color sensor object that is created. All communication
    // will occur though it.
    AdafruitColorSensor8863 activeColorSensor;

    // The next variable is needed because I need to access the led through the actual color sensor
    // object, not the objeect I am using to communicate with
    AdafruitColorSensor8863 actualColorSensor;

    // The name of the color sensor. This needs to be used when configuring the phone.
    String colorSensorName = "colorSensor";

    // The name of the core device interface module. This needs to be used when configuring the phone.
    String coreDIMName = "coreDIM";

    // Connect the wires for the led from the color sensor board to the discrete output pin on
    // channel of the core DIM listed below.
    final int CHANNEL_FOR_LED1 = 4;
    final int CHANNEL_FOR_LED2 = 5;
    final int CHANNEL_FOR_LED3 = 6;
    final int CHANNEL_FOR_LED4 = 7;

    ArrayList<AdafruitColorSensor8863.IntegrationTime> integrationTime = new ArrayList<AdafruitColorSensor8863.IntegrationTime>();
    int integrationTimeIndexColorSensor1 = 0;
    int integrationTimeIndexColorSensor2 = 0;
    int integrationTimeIndexColorSensor3 = 0;
    int integrationTimeIndexColorSensor4 = 0;
    int integrationTimeIndex;

    boolean leftBumperIsReleased = false;
    boolean ledState = false;

    int delayInMsec = 100;

    ElapsedTime timer;

    @Override
    public void runOpMode() {

        integrationTime.add(AdafruitColorSensor8863.IntegrationTime.AMS_COLOR_ITIME_24MS);
        integrationTime.add(AdafruitColorSensor8863.IntegrationTime.AMS_COLOR_ITIME_101MS);
        integrationTime.add(AdafruitColorSensor8863.IntegrationTime.AMS_COLOR_ITIME_307MS);
        integrationTime.add(AdafruitColorSensor8863.IntegrationTime.AMS_COLOR_ITIME_537MS);
        integrationTime.add(AdafruitColorSensor8863.IntegrationTime.AMS_COLOR_ITIME_700MS);

        timer = new ElapsedTime();

        boolean dpad_upIsReleased = false;

        // Create an I2C mux and initialize it
        mux = new AdafruitI2CMux(hardwareMap, muxName, muxAddress);
        // disconnect all 8 ports
        mux.disablePorts();

        colorSensorCommunicator = new I2cDeviceCommunicator(hardwareMap, "colorSensor", AdafruitColorSensor8863.getAdafruitColorSensor8863I2cAddress());

        // connect only port 0. A color sensor (colorSensor1) has been wired to that port of the mux.
        mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT0);
        // create colorSensor1 and initialize it
        colorSensor1 = AdafruitColorSensor8863.createAdaFruitColorSensor8863ForMux(hardwareMap, colorSensorCommunicator, coreDIMName, CHANNEL_FOR_LED1);
        activeColorSensor = colorSensor1;
        activeColorSensor.reportStatus("Color Sensor 1", telemetry);

        // Note that there are 4 color sensor objects. But the phone and hardware map only know about
        // one. That is because you are switching between them using the mux. I'm not sure if you could
        // get away with only 1 color sensor object but since I need to initialize both color sensors
        // it is easier to use 4 objects.

        // connect only port 1. A color sensor (colorSensor2) has been wired to that port of the mux.
        mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT1);
        // create colorSensor and initialize it
        colorSensor2 = AdafruitColorSensor8863.createAdaFruitColorSensor8863ForMux(hardwareMap, colorSensorCommunicator, coreDIMName, CHANNEL_FOR_LED2);
        activeColorSensor = colorSensor2;
        integrationTimeIndex = integrationTimeIndexColorSensor2;
        activeColorSensor.reportStatus("Color Sensor 2", telemetry);


        // connect only port 3. A color sensor (colorSensor3) has been wired to that port of the mux.
        mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT2);
        // create colorSensor and initialize it
        colorSensor3 = AdafruitColorSensor8863.createAdaFruitColorSensor8863ForMux(hardwareMap, colorSensorCommunicator, coreDIMName, CHANNEL_FOR_LED3);
        activeColorSensor = colorSensor3;
        integrationTimeIndex = integrationTimeIndexColorSensor3;
        activeColorSensor.reportStatus("Color Sensor 3", telemetry);


        // connect only port 4. A color sensor (colorSensor4) has been wired to that port of the mux.
        mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT3);
        // create colorSensor and initialize it
        colorSensor4 = AdafruitColorSensor8863.createAdaFruitColorSensor8863ForMux(hardwareMap, colorSensorCommunicator, coreDIMName, CHANNEL_FOR_LED4);
        activeColorSensor = colorSensor4;
        integrationTimeIndex = integrationTimeIndexColorSensor4;
        activeColorSensor.reportStatus("Color Sensor 4", telemetry);

        // connect only port 0. A color sensor has been wired to that port.
        mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT0);
        actualColorSensor = colorSensor1;
        integrationTimeIndex = integrationTimeIndexColorSensor1;

        // Wait for the start button
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        waitForStart();

        // Put your calls here - they will not run in a loop

        while (opModeIsActive()) {

            // Use Left bumpre to toggle the led on or off
            if (gamepad1.left_bumper) {
                if (leftBumperIsReleased) {
                    activeColorSensor.toggleLED();
                    leftBumperIsReleased = false;
                }
            } else {
                leftBumperIsReleased = true;
            }

            // Use gamepad Y to select the color sensor on port 0
            if (gamepad1.y) {
                mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT0);
                activeColorSensor = colorSensor1;
                colorSensorName = "Color Sensor 1";
                integrationTimeIndex = integrationTimeIndexColorSensor1;
            }

            // Use gamepad x to select the color sensor on port 1
            // I'm using the alternative set of methods to select and enable a port just to
            // show how they are used.
            if (gamepad1.x) {
                mux.disablePorts();
                mux.selectPort(AdafruitI2CMux.PortNumber.PORT1);
                mux.enablePorts();
                activeColorSensor = colorSensor2;
                colorSensorName = "Color Sensor 2";
            }

            // Use gamepad a to select the color sensor on port 2
            if (gamepad1.a) {
                mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT2);
                activeColorSensor = colorSensor3;
                colorSensorName = "Color Sensor 3";
            }

            // Use gamepad B to select the color sensor on port 3
            if (gamepad1.b) {
                mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT3);
                activeColorSensor = colorSensor4;
                colorSensorName = "Color Sensor 4";
            }

            // Use gamepad right bumper to disable all ports
            if (gamepad1.right_bumper) {
                mux.disablePorts();
            }

            // Change the integration time for this color sensor
            if (gamepad1.dpad_up) {
                if (dpad_upIsReleased) {
                    incrementIntegrationTime();
                }
                dpad_upIsReleased = false;
            } else {
                dpad_upIsReleased = true;
            }

            //Display the current values from the sensor
            telemetry.addData("Port = ", mux.getActivePortAsString());
            telemetry.addData("Integration time = ", integrationTime.get(integrationTimeIndex).toString());
            activeColorSensor.reportStatus(colorSensorName, telemetry);
            activeColorSensor.displayColorSensorData(telemetry);

            // turn on the led in the core DIM that matches the color read from the sensor - as long
            // as the color is red or blue :-)
            if (activeColorSensor.isBlueUsingRGB()) {
                activeColorSensor.turnCoreDIMBlueLEDOn();
                activeColorSensor.turnCoreDIMRedLEDOff();
            }
            if (activeColorSensor.isRedUsingRGB()) {
                activeColorSensor.turnCoreDIMRedLEDOn();
                activeColorSensor.turnCoreDIMBlueLEDOff();
            }

            idle();
        }

        // Put your cleanup code here - it runs as the application shuts down
        mux.disablePorts();
        telemetry.addData(">", "Done");
        telemetry.update();

    }

    void delay(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void incrementIntegrationTime() {
        // get the current index corresponding to the setting on the active color sensor
        AdafruitColorSensor8863.IntegrationTime currentIntegrationTime = activeColorSensor.getIntegrationTime();
        integrationTimeIndex = integrationTime.indexOf(currentIntegrationTime);
        integrationTimeIndex++;
        if (integrationTimeIndex > 4) {
            integrationTimeIndex = 0;
        }
        switch (integrationTime.get(integrationTimeIndex)) {
            case AMS_COLOR_ITIME_24MS:
                activeColorSensor.setIntegrationTime24ms();
                break;
            case AMS_COLOR_ITIME_101MS:
                activeColorSensor.setIntegrationTime101ms();
                break;
            case AMS_COLOR_ITIME_307MS:
                activeColorSensor.setIntegrationTime307ms();
                break;
            case AMS_COLOR_ITIME_537MS:
                activeColorSensor.setIntegrationTime307ms();
                break;
            case AMS_COLOR_ITIME_700MS:
                activeColorSensor.setIntegrationTime700ms();
                break;
        }
    }
}
