package org.firstinspires.ftc.teamcode.opmodes.GenericTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitColorSensor;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitColorSensor8863;

/**
 * This opmode can be used to test the Adafruit color sensor using AdafruitColorSensor8863 as the
 * driver. Note that if you have more than one color sensor you will have to use an I2C mux since
 * the address for this color sensor is fixed and you can't have two sensors with the same address
 * on the bus.
 *
 */
@TeleOp(name = "Test Adafruit Color Sensor 8863", group = "Test")
//@Disabled
public class TestAdafruitColorSensor8863 extends LinearOpMode {

    // Put your variable declarations here
    int red = 0;
    int green = 0;
    int blue = 0;

    // You connect a wire from the pin on the circuit board labeled LED to the SIGNAL pin on a
    // core DIM digital I/O port. If you don't do this, no biggie. The LED will just stay on
    // all the time.
    final int CHANNEL_FOR_LED = 5;
    // configure your phone with this name for the core device interface module
    final String coreDIMName = "coreDIM";
    // configure your phone for an I2C Device type with this name
    final String colorSensorName = "colorSensor";

    boolean xButtonIsReleased = false;
    boolean yButtonIsReleased = false;
    boolean aButtonIsReleased = false;
    boolean bButtonIsReleased = false;

    AdafruitColorSensor8863 colorSensor;

    boolean ledState = false;

    ElapsedTime timer;

    @Override
    public void runOpMode() {


        // Put your initializations here
        colorSensor = AdafruitColorSensor8863.createAdaFruitColorSensor8863Single(hardwareMap, colorSensorName,
                coreDIMName, CHANNEL_FOR_LED);

        timer = new ElapsedTime();
        
        // Wait for the start button
        telemetry.addData(">", "Press Start to run" );
        telemetry.update();
        waitForStart();
        timer.reset();

        // Put your calls here - they will not run in a loop

        while(opModeIsActive()) {

            // Put your calls that need to run in a loop here
            // Use gamepad X to toggle the led on or off
            if (gamepad1.x) {
                if (xButtonIsReleased) {
                    if (!ledState) {
                        colorSensor.turnLEDOn();
                        ledState = true;
                    } else {
                        colorSensor.turnLEDOff();
                        ledState = false;
                    }
                    xButtonIsReleased = false;
                }
            } else {
                xButtonIsReleased = true;
            }

            // Display the current values from the sensor
            colorSensor.displayColorSensorData(telemetry);
            
            idle();
        }

        // Put your cleanup code here - it runs as the application shuts down
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
