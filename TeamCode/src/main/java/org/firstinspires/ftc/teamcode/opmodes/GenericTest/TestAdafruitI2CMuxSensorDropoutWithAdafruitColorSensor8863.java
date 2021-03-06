package org.firstinspires.ftc.teamcode.opmodes.GenericTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitColorSensor8863;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitI2CMux;

/**
 * This opmode demonstates how to use the AdafruitI2CMux class to talk to an I2C mux.
 * It assumes that you have 4 adafruit color sensors connected on these ports of the mux (not the
 * core device interface module):
 * colorSensor1 - port 0
 * colorSensor2 - port 1
 * colorSensor3 - port 2
 * colorSensor4 - port 4
 * The phone needs to be configured with an "I2C Device" on the core device interface module port
 * that the mux is connected to. The name of the mux is "mux".
 * It also needs an Adafruit Color Sensor configured on one of the other I2C ports of the core
 * device interface module. It does not matter which port. The name of the color sensor is
 * "colorSensor". The type of sensor is "I2C Device".
 * Lastly, the core device interface module needs to be named "coreDIM" in your phone configuration.
 * <p>
 * The purpose of this opmode is to check if a color sensor attached to the mux ever disconnects
 * spontaneously. Or you can disconnect the sensor by pulling the SDA and SCL pins from the sensor
 * and make sure that the code detects that the sensor has gone away.
 */
@TeleOp(name = "Test Adafruit I2C Mux Sensor dropout", group = "Test")
@Disabled
public class TestAdafruitI2CMuxSensorDropoutWithAdafruitColorSensor8863 extends LinearOpMode {

    // Put your variable declarations here
    AdafruitI2CMux mux;
    // The name of the mux. This needs to be used when configuring the phone.
    String muxName = "mux";
    // The default address of the I2C mux. If you change that, then you have to change this address.
    byte muxAddress = 0x70;

    // A color sensor for use in testing the mux. This is my class and is wrapped around ColorSensor.
    // This class integrates both the color sensor and the led control using a discrete output port.
    AdafruitColorSensor8863 colorSensor1;
    AdafruitColorSensor8863 colorSensor2;
    AdafruitColorSensor8863 colorSensor3;
    AdafruitColorSensor8863 colorSensor4;

    // The next variable will point to one of the two colorSensors above; the one that is active
    AdafruitColorSensor8863 activeColorSensor;

    // The name of the color sensor. This needs to be used when configuring the phone.
    String colorSensorName = "colorSensor";

    // The name of the core device interface module. This needs to be used when configuring the phone.
    String coreDIMName = "coreDIM";

    final int CHANNEL_FOR_LED1 = 1;
    final int CHANNEL_FOR_LED2 = 2;
    final int CHANNEL_FOR_LED3 = 3;
    final int CHANNEL_FOR_LED4 = 4;

    // a timer for use in testing the switching
    ElapsedTime timer;

    // counters for failure and success
    int portOffSensorPresentCount = 0;
    int portOffSensorNotPresentCount = 0;
    int portOnSuccessCount = 0;
    int portOnFailureCount = 0;

    int timeBetweenSwitchesInMSec = 10;

    int delayInMsec = 0;

    String buffer;

    // controls the number of color sensors attached to the mux. This can range from 1 to 4.
    int numberOfColorSensors = 4;

    @Override
    public void runOpMode() {

        // Create the timer
        timer = new ElapsedTime();

        // Create an I2C mux and initialize it
        mux = new AdafruitI2CMux(hardwareMap, muxName, muxAddress);
        // disconnect all 8 ports
        mux.disablePorts();

        // connect only port 0. A color sensor (colorSensor1) has been wired to that port of the mux.
        mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT0);
        // create colorSensor1 and initialize it
        colorSensor1 = new AdafruitColorSensor8863(hardwareMap, colorSensorName, coreDIMName, CHANNEL_FOR_LED1);
        // report whether the color sensor id and data are valid
        activeColorSensor = colorSensor1;
        activeColorSensor.reportStatus("Color Sensor 1", telemetry);

        if (numberOfColorSensors >= 2) {
            // connect only port 1. A color sensor (colorSensor2) has been wired to that port of the mux.
            mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT1);
            // create colorSensor1 and initialize it
            colorSensor2 = new AdafruitColorSensor8863(hardwareMap, colorSensorName, coreDIMName, CHANNEL_FOR_LED2);
            // report whether the color sensor id and data are valid
            activeColorSensor = colorSensor2;
            activeColorSensor.reportStatus("Color Sensor 2", telemetry);
        }

        if (numberOfColorSensors >= 3) {
            // connect only port 2. A color sensor (colorSensor2) has been wired to that port of the mux.
            mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT2);
            // create colorSensor1 and initialize it
            colorSensor3 = new AdafruitColorSensor8863(hardwareMap, colorSensorName, coreDIMName, CHANNEL_FOR_LED3);
            // report whether the color sensor id and data are valid
            activeColorSensor = colorSensor3;
            activeColorSensor.reportStatus("Color Sensor 3", telemetry);
        }

        if (numberOfColorSensors >= 4) {
            // connect only port 4. A color sensor (colorSensor2) has been wired to that port of the mux.
            mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT3);
            // create colorSensor1 and initialize it
            colorSensor4 = new AdafruitColorSensor8863(hardwareMap, colorSensorName, coreDIMName, CHANNEL_FOR_LED4);
            // report whether the color sensor id and data are valid
            activeColorSensor = colorSensor4;
            activeColorSensor.reportStatus("Color Sensor 4", telemetry);
        }

        // Wait for the start button
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        waitForStart();

        timer.reset();

        // Put your calls here - they will not run in a loop

        while (opModeIsActive()) {

            if (timer.milliseconds() > timeBetweenSwitchesInMSec) {

                // switch all of the ports off
                mux.disablePorts();
                // check to see the color sensor is not responding
                if (activeColorSensor.checkDeviceId()) {
                    // uh oh the color sensor responded
                    portOffSensorPresentCount++;
                } else {
                    // it worked - no response from the color sensor
                    portOffSensorNotPresentCount++;
                }

                // switch on port
                mux.selectAndEnableAPort(AdafruitI2CMux.PortNumber.PORT0);
                // check to see the color sensor is responding
                if (activeColorSensor.checkDeviceId()) {
                    // Yes! - the color sensor responded
                    portOnSuccessCount++;
                } else {
                    // uh oh - no response from the color sensor
                    portOnFailureCount++;
                }
                // reset the timer
                timer.reset();
            }

            //Display the current values from the sensor
            buffer = portOffSensorPresentCount + "/" + portOffSensorNotPresentCount;
            telemetry.addData("Port off sensor present/not present = ", buffer);
            buffer = portOnSuccessCount + "/" + portOnFailureCount;
            telemetry.addData("Port on sensor present/not present = ", buffer);
            telemetry.update();

            idle();
        }

        // Put your cleanup code here - it runs as the application shuts down
        mux.disablePorts();
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
