package org.firstinspires.ftc.teamcode.opmodes.RelicRecoveryTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.DcMotor8863;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.JoyStick;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.Servo8863;
import org.firstinspires.ftc.teamcode.Lib.RelicRecoveryLib.NathanMagicRobot;
import org.firstinspires.ftc.teamcode.opmodes.GenericTest.RobotConfigMappingForGenericTest;

/**
 * Created by ball on 10/7/2017.
 */

@TeleOp(name = "ThreeAxisArmTest", group = "Run")
//@Disabled

public class ThreeAxisArmTest extends LinearOpMode {

    //*********************************************************************************************
    //             Declarations
    //*********************************************************************************************
    
    public DcMotor8863 rollMotor;
    public DcMotor8863 pitchMotor;
    public Servo8863 yawServo;
    
    // for use in debouncing the button. A long press will only result in one transition of the
    // button

    boolean gamepad1RightBumperIsReleased = true;
    boolean gamepad1LeftBumperIsReleased = true;
    boolean gamepad1aButtonIsReleased = true;
    boolean gamepad1bButtonIsReleased = true;
    boolean gamepad1yButtonIsReleased = true;
    boolean gamepad1xButtonIsReleased = true;
    boolean gamepad1DpadUpIsReleased = true;
    boolean gamepad1DpadDownIsReleased = true;
    boolean gamepad1DpadLeftIsReleased = true;
    boolean gamepad1DpadRightIsReleased = true;
    boolean gamepad1LeftStickButtonIsReleased = true;
    boolean gamepad1RightStickButtonIsReleased = true;

    // joystick and joystick value declarations - game pad 1
    final static double JOYSTICK_DEADBAND_VALUE = .15;
    final static double JOYSTICK_HALF_POWER = .5;
    final static double JOYSTICK_QUARTER_POWER = .25;

    JoyStick gamepad1LeftJoyStickX;
    JoyStick gamepad1LeftJoyStickY;
    double gamepad1LeftJoyStickXValue = 0;
    double gamepad1LeftJoyStickYValue = 0;

    JoyStick gamepad1RightJoyStickX;
    JoyStick gamepad1RightJoyStickY;
    double gamepad1RightJoyStickXValue = 0;
    double gamepad1RightJoyStickYValue = 0;

    // for use in debouncing the button. A long press will only result in one transition of the
    // button
    boolean gamepad2RightBumperIsReleased = true;
    boolean gamepad2LeftBumperIsReleased = true;
    boolean gamepad2aButtonIsReleased = true;
    boolean gamepad2bButtonIsReleased = true;
    boolean gamepad2yButtonIsReleased = true;
    boolean gamepad2xButtonIsReleased = true;
    boolean gamepad2DpadUpIsReleased = true;
    boolean gamepad2DpadDownIsReleased = true;
    boolean gamepad2DpadLeftIsReleased = true;
    boolean gamepad2DpadRightIsReleased = true;
    boolean gamepad2LeftStickButtonIsReleased = true;
    boolean gamepad2RightStickButtonIsReleased = true;

    // joystick and joystick value declarations - game pad 2
    JoyStick gamepad2LeftJoyStickX;
    JoyStick gamepad2LeftJoyStickY;
    double gamepad2LeftJoyStickXValue = 0;
    double gamepad2LeftJoyStickYValue = 0;

    JoyStick gamepad2RightJoyStickX;
    JoyStick gamepad2RightJoyStickY;
    double gamepad2RightJoyStickXValue = 0;
    double gamepad2RightJoyStickYValue = 0;

    // drive train powers for tank drive
    double leftPower = 0;
    double rightPower = 0;

    // drive train powers for differential drive
    double throttle = 0;
    double direction = 0;

    // lift motor power
    double liftMotorPower = 0;
    double actualLiftMotorPower = 0;

    @Override
    public void runOpMode() {

        //*********************************************************************************************
        //  Initializations after the pogram is selected by the user on the driver phone
        //*********************************************************************************************
        

//        rollMotor = new DcMotor8863("rollMotor", hardwareMap);
//        rollMotor.setMotorType(DcMotor8863.MotorType.ANDYMARK_20);
//        rollMotor.setMovementPerRev(360);
//        rollMotor.setTargetEncoderTolerance(10);
//        rollMotor.setFinishBehavior(DcMotor8863.FinishBehavior.HOLD);
//        rollMotor.setMotorMoveType(DcMotor8863.MotorMoveType.RELATIVE);
//        rollMotor.setMinMotorPower(-1);
//        rollMotor.setMaxMotorPower(1);
//        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        pitchMotor = new DcMotor8863("pitchMotor", hardwareMap);
//        pitchMotor.setMotorType(DcMotor8863.MotorType.ANDYMARK_20);
//        pitchMotor.setMovementPerRev(360);
//        pitchMotor.setTargetEncoderTolerance(10);
//        pitchMotor.setFinishBehavior(DcMotor8863.FinishBehavior.HOLD);
//        pitchMotor.setMotorMoveType(DcMotor8863.MotorMoveType.RELATIVE);
//        pitchMotor.setMinMotorPower(-1);
//        pitchMotor.setMaxMotorPower(1);
//        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Game Pad 1 joysticks
        gamepad1LeftJoyStickX = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.NO_INVERT_SIGN);
        gamepad1LeftJoyStickY = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.INVERT_SIGN);

        gamepad1RightJoyStickX = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.NO_INVERT_SIGN);
        gamepad1RightJoyStickY = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.INVERT_SIGN);

        // Game Pad 2 joysticks
        gamepad2LeftJoyStickX = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.NO_INVERT_SIGN);
        gamepad2LeftJoyStickY = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.INVERT_SIGN);

        gamepad2RightJoyStickX = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.NO_INVERT_SIGN);
        gamepad2RightJoyStickY = new JoyStick(JoyStick.JoyStickMode.SQUARE, JOYSTICK_DEADBAND_VALUE, JoyStick.InvertSign.INVERT_SIGN);

        // Wait for the start button
        telemetry.addData(">", "Press start to run Teleop");
        telemetry.update();
        waitForStart();

        //*********************************************************************************************
        //             Robot Running after the user his play on the driver phone
        //*********************************************************************************************

        while (opModeIsActive()) {

            //*************************************************************************************
            // Gamepad 1 buttons
            //*************************************************************************************

            if (gamepad1.y) {
                if (gamepad1yButtonIsReleased) {
                    gamepad1yButtonIsReleased = false;
                }
            } else {
                gamepad1yButtonIsReleased = true;
            }

            if (gamepad1.a) {
                if (gamepad1aButtonIsReleased) {
                    gamepad1aButtonIsReleased = false;
                }
            } else {
                gamepad1aButtonIsReleased = true;
            }

            if (gamepad1.x) {
                if (gamepad1xButtonIsReleased) {
                    gamepad1xButtonIsReleased = false;
                }
            } else {
                gamepad1xButtonIsReleased = true;
            }

            if (gamepad1.b) {
                if (gamepad1bButtonIsReleased) {
                    gamepad1bButtonIsReleased = false;
                }
            } else {
                gamepad1bButtonIsReleased = true;
            }

            if (gamepad1.right_bumper) {
                if (gamepad1RightBumperIsReleased) {
                    gamepad1RightBumperIsReleased = false;
                }
            } else {
                gamepad1RightBumperIsReleased = true;
            }

            if (gamepad1.left_bumper) {
                if (gamepad1LeftBumperIsReleased) {
                    gamepad1LeftBumperIsReleased = false;
                }
            } else {
                gamepad1LeftBumperIsReleased = true;
            }

            if (gamepad1.dpad_up) {
                if (gamepad1DpadUpIsReleased) {
                    gamepad1LeftJoyStickX.setFullPower();
                    gamepad1LeftJoyStickY.setFullPower();
                    gamepad1RightJoyStickX.setFullPower();
                    gamepad1RightJoyStickY.setFullPower();
                    gamepad1DpadUpIsReleased = false;
                }
            } else {
                gamepad1DpadUpIsReleased = true;
            }

            if (gamepad1.dpad_down) {
                if (gamepad1DpadDownIsReleased) {
                    gamepad1LeftJoyStickX.setQuarterPower();
                    gamepad1LeftJoyStickY.setQuarterPower();
                    gamepad1RightJoyStickX.setQuarterPower();
                    gamepad1RightJoyStickY.setQuarterPower();
                    gamepad1DpadDownIsReleased = false;
                }
            } else {
                gamepad1DpadDownIsReleased = true;
            }

            if (gamepad1.dpad_left) {
                if (gamepad1DpadLeftIsReleased) {
                    gamepad1LeftJoyStickX.setHalfPower();
                    gamepad1LeftJoyStickY.setHalfPower();
                    gamepad1RightJoyStickX.setHalfPower();
                    gamepad1RightJoyStickY.setHalfPower();
                    gamepad1DpadLeftIsReleased = false;
                }
            } else {
                gamepad1DpadLeftIsReleased = true;
            }

            if (gamepad1.dpad_right) {
                if (gamepad1DpadRightIsReleased) {
                    gamepad1LeftJoyStickX.set10PercentPower();
                    gamepad1LeftJoyStickY.set10PercentPower();
                    gamepad1RightJoyStickX.set10PercentPower();
                    gamepad1RightJoyStickY.set10PercentPower();
                    gamepad1DpadRightIsReleased = false;
                }
            } else {
                gamepad1DpadRightIsReleased = true;
            }

            if (gamepad1.left_stick_button) {
                if (gamepad1LeftStickButtonIsReleased) {
                    gamepad1LeftStickButtonIsReleased = false;
                }
            } else {
                gamepad1LeftStickButtonIsReleased = true;
            }

            if (gamepad1.right_stick_button) {
                if (gamepad1RightStickButtonIsReleased) {
                    gamepad1RightStickButtonIsReleased = false;
                }
            } else {
                gamepad1RightStickButtonIsReleased = true;
            }

            //**************************************************************************************
            // Gamepad 1 joysticks
            //**************************************************************************************

            gamepad1LeftJoyStickXValue = gamepad1LeftJoyStickX.scaleInput(gamepad1.left_stick_x);
            gamepad1LeftJoyStickYValue = gamepad1LeftJoyStickY.scaleInput(gamepad1.left_stick_y);

            gamepad1RightJoyStickXValue = gamepad1RightJoyStickX.scaleInput(gamepad1.right_stick_x);
            gamepad1RightJoyStickYValue = gamepad1RightJoyStickY.scaleInput(gamepad1.right_stick_y);

//            pitchMotor.setPower(gamepad1RightJoyStickYValue);
//            rollMotor.setPower(gamepad1RightJoyStickXValue);
            //**************************************************************************************
            // Gamepad 2 buttons
            //**************************************************************************************

            if (gamepad2.y) {
                if (gamepad2yButtonIsReleased) {
                    gamepad2yButtonIsReleased = true;
                }
            } else {
                gamepad2yButtonIsReleased = true;
            }

            if (gamepad2.a) {
                if (gamepad2aButtonIsReleased) {
                    gamepad2aButtonIsReleased = true;
                }
            } else {
                gamepad2aButtonIsReleased = true;
            }

            if (gamepad2.x) {
                if (gamepad2xButtonIsReleased) {
                    gamepad2xButtonIsReleased = false;
                }
            } else {
                gamepad2xButtonIsReleased = true;
            }

            if (gamepad2.b) {
                if (gamepad2bButtonIsReleased) {
                    gamepad2bButtonIsReleased = true;
                }
            } else {
                gamepad2bButtonIsReleased = true;
            }

            if (gamepad2.right_bumper) {
                if (gamepad2RightBumperIsReleased) {
                    gamepad2RightBumperIsReleased = false;
                }
            } else {
                gamepad2RightBumperIsReleased = true;
            }

            if (gamepad2.left_bumper) {
                if (gamepad2LeftBumperIsReleased) {
                    gamepad2LeftBumperIsReleased = false;
                }
            } else {
                gamepad2LeftBumperIsReleased = true;
            }

            if (gamepad2.dpad_up) {
                if (gamepad2DpadUpIsReleased) {
                    gamepad2DpadUpIsReleased = false;
                }
            } else {
                gamepad2DpadUpIsReleased = true;
            }

            if (gamepad2.dpad_down) {
                if (gamepad2DpadDownIsReleased) {
                    gamepad2DpadDownIsReleased = false;
                }
            } else {
                gamepad2DpadDownIsReleased = true;
            }

            if (gamepad2.dpad_left) {
                if (gamepad2DpadLeftIsReleased) {
                    gamepad2DpadLeftIsReleased = false;
                }
            } else {
                gamepad2DpadLeftIsReleased = true;
            }

            if (gamepad2.dpad_right) {
                if (gamepad2DpadRightIsReleased) {
                    gamepad2DpadRightIsReleased = false;
                }
            } else {
                gamepad2DpadRightIsReleased = true;
            }

            if (gamepad2.left_stick_button) {
                if (gamepad2LeftStickButtonIsReleased) {
                    gamepad2LeftStickButtonIsReleased = false;
                }
            } else {
                gamepad2LeftStickButtonIsReleased = true;
            }

            if (gamepad2.right_stick_button) {
                if (gamepad2RightStickButtonIsReleased) {
                    gamepad2RightStickButtonIsReleased = false;
                }
            } else {
                gamepad2RightStickButtonIsReleased = true;
            }

            //**************************************************************************************
            // Gamepad 2 joysticks
            //**************************************************************************************

            gamepad2LeftJoyStickXValue = gamepad2LeftJoyStickX.scaleInput(gamepad2.left_stick_x);
            gamepad2LeftJoyStickYValue = gamepad2LeftJoyStickY.scaleInput(gamepad2.left_stick_y);

            gamepad2RightJoyStickXValue = gamepad2RightJoyStickX.scaleInput(gamepad2.right_stick_x);
            gamepad2RightJoyStickYValue = gamepad2RightJoyStickY.scaleInput(gamepad2.right_stick_y);


            //*************************************************************************************
            //  Process joysticks into drive train commands
            // ************************************************************************************

            // joysticks to tank drive
            leftPower = gamepad1LeftJoyStickYValue;
            rightPower = gamepad1RightJoyStickYValue;

            // joysticks to differential drive
            throttle = gamepad1RightJoyStickYValue;
            direction = gamepad1RightJoyStickXValue;

            // joystick for lift motor
            liftMotorPower = gamepad2RightJoyStickYValue;


            telemetry.addData("Lift Motor Command", "%3.2f", liftMotorPower);

            // Display telemetry
            telemetry.addData("Left Motor Speed = ", "%3.2f", leftPower);
            telemetry.addData("Actual Lift Motor Power", "%3.2f", actualLiftMotorPower);
            telemetry.addData("Right Motor Speed = ", "%3.2f", rightPower);
            telemetry.addData("Power Reduction = ", "%1.2f", gamepad1LeftJoyStickY.getReductionFactor());
            telemetry.addData(">", "Press Stop to end.");
            telemetry.update();

            idle();
        }

        //*************************************************************************************
        //  Stop everything after the user hits the stop button on the driver phone
        // ************************************************************************************

        // Stop has been hit, shutdown everything
        telemetry.addData(">", "Done");
        telemetry.update();
    }

    //*********************************************************************************************
    //             Helper methods
    //*********************************************************************************************
}
