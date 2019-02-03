package org.firstinspires.ftc.teamcode.opmodes.GenericTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.AdafruitIMU8863;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DataLogging;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DriveCurve;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DriveTrain;

/**
 * This Opmode is a shell for a linear OpMode. Copy this file and fill in your code as indicated.
 *
 *
 */
@TeleOp(name = "Test Drive Curve + Straight", group = "Test")
//@Disabled
public class TestDriveCurvePlusStraight extends LinearOpMode {

    // Put your variable declarations here
    DriveCurve driveCurve;
    AdafruitIMU8863 imu;
    DataLogging logfile;
    DriveTrain driveTrain;


    @Override
    public void runOpMode() {

        // Put your initializations here
        logfile = new DataLogging( "Test Drive Curve + Straight", telemetry);
        driveTrain = DriveTrain.DriveTrainAutonomous(hardwareMap, telemetry);

        double curveAngle = -90.0;
        double speed = 0.1;
        double curveRadius = 100; // cm
        double wheelbase = 37.38; // measured
        driveCurve = new DriveCurve(curveAngle, speed, curveRadius, wheelbase, driveTrain.imu, logfile, driveTrain);
        driveCurve.enableLogging();

                // Wait for the start button
        telemetry.addData(">", "Press Start to run" );
        telemetry.update();
        waitForStart();

        // Put your calls here - they will not run in a loop
//        driveTrain.setLeftDriveMotorSpeed(driveCurve.getLeftWheelSpeed());
//        driveTrain.setRightDriveMotorSpeed(driveCurve.getRightWheelSpeed());
//        driveTrain.applyPowersToMotors();
//
//        while(opModeIsActive() && !driveCurve.update()) {
//            telemetry.addData(">", "Press Stop to end test." );
//            telemetry.update();
//            idle();
//        }

        driveTrain.setupDriveUsingIMU(0, 25, 0.1, AdafruitIMU8863.AngleMode.RELATIVE);
        while (opModeIsActive()&& !driveTrain.updateDriveUsingIMU()) {
            logfile.logData(Double.toString(driveTrain.imu.getHeading()), Double.toString(driveTrain.getDistanceDriven()));
            // Display the current value
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();
            idle();
        }
        driveTrain.stopDriveUsingIMU();

        // Put your cleanup code here - it runs as the application shuts down
        logfile.closeDataLog();
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
