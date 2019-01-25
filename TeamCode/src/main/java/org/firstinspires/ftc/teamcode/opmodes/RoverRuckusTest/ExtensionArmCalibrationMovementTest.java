package org.firstinspires.ftc.teamcode.opmodes.RoverRuckusTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Lib.RoverRuckusLib.CollectorArm;
import org.firstinspires.ftc.teamcode.Lib.RoverRuckusLib.DeliveryLiftSystem;

/**
 * This Opmode is a shell for a linear OpMode. Copy this file and fill in your code as indicated.
 */
@TeleOp(name = "Extension Arm Calibration Movement Test", group = "Test")
//@Disabled
public class ExtensionArmCalibrationMovementTest extends LinearOpMode {

    // Put your variable declarations here
    public CollectorArm collectorArm;

    @Override
    public void runOpMode() {


        // Put your initializations here
        collectorArm = new CollectorArm(hardwareMap, telemetry);
        collectorArm.enableDebugMode();
        collectorArm.init();

        // Wait for the start button
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        waitForStart();

        // Put your calls here - they will not run in a loop
        // reduces the power applied to the motor
        collectorArm.enableDebugMode();
        collectorArm.moveExtensionArmTwoInchesOut();

        while (opModeIsActive()) {

            // Put your calls that need to run in a loop here
            collectorArm.update();

            // Display the current value
            collectorArm.displayExtensionMotorEncoder();
            collectorArm.displayExtensionArmPosition();
            collectorArm.displayExtensionArmState();
            telemetry.addData(">", "Press Stop to end test.");

            telemetry.update();

            idle();
        }

        // Put your cleanup code here - it runs as the application shuts down
        telemetry.addData(">", "Done");
        collectorArm.displayExtensionMotorEncoder();
        collectorArm.displayExtensionArmPosition();
        collectorArm.displayExtensionArmState();
        telemetry.update();
        sleep(4000);
    }
}
