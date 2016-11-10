package org.firstinspires.ftc.teamcode.Lib.FTCLib;


import com.qualcomm.robotcore.util.ElapsedTime;

public class RampControl {

    //*********************************************************************************************
    //          ENUMERATED TYPES
    //
    // user defined types
    //
    //*********************************************************************************************


    //*********************************************************************************************
    //          PRIVATE DATA FIELDS
    //
    // can be accessed only by this class, or by using the public
    // getter and setter methods
    //*********************************************************************************************

    private ElapsedTime timer;

    private double initialValue = 0;

    private double finalValue = 0;

    private double timeToReachFinalValueInmSec = 0;

    //*********************************************************************************************
    //          GETTER and SETTER Methods
    //
    // allow access to private data fields for example setMotorPower,
    // getPositionInTermsOfAttachment
    //*********************************************************************************************

    public void setInitialValue(double initialValue) {
        this.initialValue = initialValue;
    }

    public void setFinalValue(double finalValue) {
        this.finalValue = finalValue;
    }

    public void setTimeToReachFinalValueInmSec(double timeToReachFinalValueInmSec) {
        this.timeToReachFinalValueInmSec = timeToReachFinalValueInmSec;
    }

    //*********************************************************************************************
    //          Constructors
    //
    // the function that builds the class when an object is created
    // from it
    //*********************************************************************************************

    public RampControl(double initialValue, double finalValue, double timeToReachFinalValueInmSec) {
        this.initialValue = initialValue;
        this.finalValue = finalValue;
        this.timeToReachFinalValueInmSec = timeToReachFinalValueInmSec;
        timer = new ElapsedTime();
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

    public void start() {
        timer.reset();
    }

    public double getRampValueLinear(double value) {

        double valueFromRamp;

        if (timer.milliseconds() < timeToReachFinalValueInmSec) {
            // if timer is less than time to reach the final value (so ramp should not be in effect)
            // then calculate the value given by the ramp (valueFromRamp = m * elapsed time + initialValue) (y=mx+b)
            valueFromRamp = (finalValue - initialValue) / timeToReachFinalValueInmSec * timer.milliseconds() + initialValue;
            // if the value input is less than the calculated ramp value then use the input, otherwise use the ramp value
            if (value < valueFromRamp) {
                return value;
            } else {
                return valueFromRamp;
            }
        } else {
            // if the time is longer than the specified time, then just return the input
            return value;
        }
    }
}
