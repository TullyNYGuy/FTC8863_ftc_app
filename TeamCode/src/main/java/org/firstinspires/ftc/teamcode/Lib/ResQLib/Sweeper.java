package org.firstinspires.ftc.teamcode.Lib.ResQLib;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Lib.FTCLib.DcMotor8863;

public class Sweeper {

    //*********************************************************************************************
    //          ENUMERATED TYPES
    //
    // user defined types
    //
    //*********************************************************************************************

    private enum SweeperState {
        FORWARD, BACKWARD, STOP
    }

    //*********************************************************************************************
    //          PRIVATE DATA FIELDS
    //
    // can be accessed only by this class, or by using the public
    // getter and setter methods
    //*********************************************************************************************

    private DcMotor8863 sweeperMotor;
    private double reversePower = -.8;
    private double forwardPower = .8;
    private SweeperState sweeperState = SweeperState.STOP;


    //*********************************************************************************************
    //          GETTER and SETTER Methods
    //
    // allow access to private data fields for example setMotorPower,
    // getPositionInTermsOfAttachment
    //*********************************************************************************************


    //*********************************************************************************************
    //          Constructors
    //
    // the function that builds the class when an object is created
    // from it
    //*********************************************************************************************

    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        sweeperMotor = new DcMotor8863(RobotConfigMapping.getSweeperMotorName(), hardwareMap);
        sweeperMotor.setMotorType(DcMotor8863.MotorType.ANDYMARK_20);
        sweeperMotor.setMovementPerRev(360);
        sweeperMotor.setTargetEncoderTolerance(5);
        sweeperMotor.setFinishBehavior(DcMotor8863.FinishBehavior.FLOAT);
        sweeperMotor.setMotorMoveType(DcMotor8863.MotorMoveType.RELATIVE);
        sweeperMotor.setMinMotorPower(-1);
        sweeperMotor.setMaxMotorPower(1);
        sweeperMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    //*********************************************************************************************
    //          Helper Methods
    //
    // methods that aid or support the major functions in the class
    //*********************************************************************************************


    //*********************************************************************************************
    //          MAJOR METHODS
    //
    // public methods that give the class its functionality
    //*********************************************************************************************

    public void stop() {
        sweeperMotor.stop();
        sweeperState = SweeperState.STOP;
    }

    public void forward() {
        sweeperMotor.runAtConstantSpeed(forwardPower);
        sweeperState = SweeperState.FORWARD;

    }

    public void backward() {
        sweeperMotor.runAtConstantSpeed(reversePower);
        sweeperState = SweeperState.BACKWARD;

    }

    public void updateSweeper() {
        if (sweeperMotor.update() == DcMotor8863.MotorState.STALLED && sweeperState == SweeperState.FORWARD) {
            sweeperMotor.rotateNumberOfDegrees(reversePower, 100, DcMotor8863.FinishBehavior.HOLD);
        }

    }

}
