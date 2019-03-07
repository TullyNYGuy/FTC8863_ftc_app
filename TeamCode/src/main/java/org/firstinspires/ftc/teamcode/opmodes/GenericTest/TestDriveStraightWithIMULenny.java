package org.firstinspires.ftc.teamcode.opmodes.GenericTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitIMU8863;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DataLogging;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DcMotor8863;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DriveTrain;

/**
 * This Opmode is a shell for a linear OpMode. Copy this file and fill in your code as indicated.
 */
@TeleOp(name = "Test Driving Straight With an IMU", group = "Test")
//@Disabled
public class TestDriveStraightWithIMULenny extends LinearOpMode {

    // Put your variable declarations here
    public DriveTrain driveTrain;
    public DataLogging logFile;

    @Override
    public void runOpMode() {


        // Put your initializations here
        driveTrain = DriveTrain.DriveTrainAutonomous(hardwareMap, telemetry);
        logFile = new DataLogging("Test Driving Straight With An IMU", telemetry);
        driveTrain.setLogFile(logFile);
        driveTrain.enableLogDrive();
        // Wait for the start button
        telemetry.addData(">", "Press Start to run");
        telemetry.update();
        waitForStart();

        // Put your calls here - they will not run in a loop
        driveTrain.setupDriveUsingIMU(0, -250, 0.3, DriveTrain.DriveDirection.REVERSE, AdafruitIMU8863.AngleMode.ABSOLUTE);
        driveTrain.imu.resetAngleReferences();
        logFile.startTimer();

        while (opModeIsActive()&& !driveTrain.updateDriveUsingIMU()) {
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();
        }

        driveTrain.stopDriveUsingIMU();
        // Put your cleanup code here - it runs as the application shuts down
        telemetry.addData(">", "Done");
        telemetry.update();
        sleep(2000);
        driveTrain.updateDriveDistance();
        logFile.logData("Final postion afer stop: Heading = " + Double.toString(driveTrain.imu.getHeading()) + " distance = " + Double.toString(driveTrain.getDistanceDriven()));
        logFile.closeDataLog();
    }
}
