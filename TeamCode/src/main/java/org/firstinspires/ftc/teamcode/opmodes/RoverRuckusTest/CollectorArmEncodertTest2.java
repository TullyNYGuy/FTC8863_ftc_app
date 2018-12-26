package org.firstinspires.ftc.teamcode.opmodes.RoverRuckusTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.DataLogging;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DcMotor8863;
import org.firstinspires.ftc.teamcode.Lib.RoverRuckusLib.CollectorArm;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevExtensions2;

/**
 * This Opmode is a shell for a linear OpMode. Copy this file and fill in your code as indicated.
 *
 *
 */
@TeleOp(name = "Collector Arm Encoder Test REAL", group = "Test")
//@Disabled
public class CollectorArmEncodertTest2 extends LinearOpMode {

    // Put your variable declarations here
    public CollectorArm collectorArm;
    public DataLogging dataLogging;
    ExpansionHubMotor motor0, motor1, motor2, motor3;
    ExpansionHubEx expansionHub;

    @Override
    public void runOpMode() {


        // Put your initializations here
        collectorArm = new CollectorArm(hardwareMap,telemetry);
        dataLogging = new DataLogging("current",telemetry);
        RevExtensions2.init();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        motor1 = (ExpansionHubMotor) hardwareMap.dcMotor.get("collectorArmRotationMotor");
        // Wait for the start button
        telemetry.addData(">", "Press Start to run" );
        telemetry.update();
        waitForStart();

        // Put your calls here - they will not run in a loop
        collectorArm.rotationGoToHome();
        while(opModeIsActive() && collectorArm.update()!= DcMotor8863.MotorState.COMPLETE_HOLD) {

            // Put your calls that need to run in a loop here
            collectorArm.displayEncoder();
            telemetry.addData("M1 current", motor1.getCurrentDraw());
            dataLogging.logData(String.format("%f5.3",motor1.getCurrentDraw()),String.format("%f5.3",collectorArm.rotationMotor.getPositionInTermsOfAttachment()));
            // Display the current value
            //telemetry.addData("Motor Speed = ", "%5.2f", powerToRunAt);
            //telemetry.addData("Encoder Count=", "%5d", motor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );

            telemetry.update();

            idle();
        }
        collectorArm.rotationGoToCollect();
        while(opModeIsActive() && collectorArm.update()!= DcMotor8863.MotorState.COMPLETE_HOLD) {

            // Put your calls that need to run in a loop here
            collectorArm.displayEncoder();
            telemetry.addData("M1 current", motor1.getCurrentDraw());
            dataLogging.logData(String.format("%f5.3",motor1.getCurrentDraw()),String.format("%f5.3",collectorArm.rotationMotor.getPositionInTermsOfAttachment()));
            // Display the current value
            //telemetry.addData("Motor Speed = ", "%5.2f", powerToRunAt);
            //telemetry.addData("Encoder Count=", "%5d", motor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );

            telemetry.update();

            idle();
        }
        collectorArm.rotationFloatArm();
        collectorArm.rotationGoToTransfer();
        while(opModeIsActive() && collectorArm.update()!= DcMotor8863.MotorState.COMPLETE_HOLD) {

            // Put your calls that need to run in a loop here
            collectorArm.displayEncoder();
            telemetry.addData("M1 current", motor1.getCurrentDraw());
            dataLogging.logData(String.format("%f5.3",motor1.getCurrentDraw()),String.format("%f5.3",collectorArm.rotationMotor.getPositionInTermsOfAttachment()));
            // Display the current value
            //telemetry.addData("Motor Speed = ", "%5.2f", powerToRunAt);
            //telemetry.addData("Encoder Count=", "%5d", motor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );

            telemetry.update();

            idle();
        }
        collectorArm.rotationGoToHome();
        while(opModeIsActive() && collectorArm.update()!= DcMotor8863.MotorState.COMPLETE_HOLD) {

            // Put your calls that need to run in a loop here
            collectorArm.displayEncoder();
            telemetry.addData("M1 current", motor1.getCurrentDraw());
            dataLogging.logData(String.format("%f5.3",motor1.getCurrentDraw()),String.format("%f5.3",collectorArm.rotationMotor.getPositionInTermsOfAttachment()));
            // Display the current value
            //telemetry.addData("Motor Speed = ", "%5.2f", powerToRunAt);
            //telemetry.addData("Encoder Count=", "%5d", motor.getCurrentPosition());
            telemetry.addData(">", "Press Stop to end test." );

            telemetry.update();

            idle();
        }
        collectorArm.rotationFloatArm();

        // Put your cleanup code here - it runs as the application shuts down
        telemetry.addData(">", "Done");
        dataLogging.closeDataLog();
        telemetry.update();

    }
}
