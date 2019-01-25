package org.firstinspires.ftc.teamcode.opmodes.RoverRuckusTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.DcMotor8863;
import org.firstinspires.ftc.teamcode.Lib.RoverRuckusLib.CollectorArm;

/**
 * This Opmode is a shell for a linear OpMode. Copy this file and fill in your code as indicated.
 *
 *
 */
@TeleOp(name = "Rotation Arm Encoder Test", group = "Test")
//@Disabled
public class RotationArmEncoderTest extends LinearOpMode {

    // Put your variable declarations here
    public CollectorArm collectorArm;

    @Override
    public void runOpMode() {


        // Put your initializations here
        //collectorArmRotationMotor = hardwareMap.get(DcMotor.class,"collectorArmRotatorMotor");
//        collectorArmRotationMotor = new DcMotor8863("collectorArmRotationMotor", hardwareMap, telemetry);
//        collectorArmRotationMotor.setMotorType(DcMotor8863.MotorType.ANDYMARK_60);
//        collectorArmRotationMotor.setMovementPerRev(360 * 48/128);
//        collectorArmRotationMotor.setMotorToHold();

        collectorArm = new CollectorArm(hardwareMap, telemetry);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run" );
        telemetry.update();
        waitForStart();

        // Put your calls here - they will not run in a loop

        while(opModeIsActive()) {

            // Put your calls that need to run in a loop here
            collectorArm.displayRotationArmEncoder();
            telemetry.addData(">", "Press Stop to end test." );

            telemetry.update();
            
            idle();
        }

        // Put your cleanup code here - it runs as the application shuts down
        collectorArm.displayRotationArmEncoder();
        telemetry.addData(">", "Done");
        telemetry.update();
        sleep(4000);

    }
}
