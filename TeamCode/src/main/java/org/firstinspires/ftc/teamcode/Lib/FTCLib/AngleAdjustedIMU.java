package org.firstinspires.ftc.teamcode.Lib.FTCLib;


public class AngleAdjustedIMU {

    //*********************************************************************************************
    //          ENUMERATED TYPES
    //
    // user defined types
    //
    //*********************************************************************************************

    /**
     * The IMU returns angles in a range from -180 to + 180 normally. But sometimes you may want
     * angles in the range of 0 to 360 instead. This enum allow you to pick one or the other.
     */
    public enum AngleRange {
        PLUS_TO_MINUS_180,
        ZERO_TO_PLUS_360,
        ZERO_TO_MINUS_360
    }

    //*********************************************************************************************
    //          PRIVATE DATA FIELDS
    //
    // can be accessed only by this class, or by using the public
    // getter and setter methods
    //*********************************************************************************************

    private double triggerAngle = 0;

    private AngleRange angleRange = AngleRange.PLUS_TO_MINUS_180;

    public AdafruitIMU8863 imu;


    //*********************************************************************************************
    //          GETTER and SETTER Methods
    //
    // allow access to private data fields for example setMotorPower,
    // getPositionInTermsOfAttachment
    //*********************************************************************************************

    public double getTriggerAngle() {
        return triggerAngle;
    }

    public void setTriggerAngle(double triggerAngle) {
        this.triggerAngle = triggerAngle;
    }

    public AngleRange getAngleRange() {
        return angleRange;
    }

    //*********************************************************************************************
    //          Constructors
    //
    // the function that builds the class when an object is created
    // from it
    //*********************************************************************************************

    public AngleAdjustedIMU(AdafruitIMU8863 imu) {
        this.imu = imu;
        angleRange = AngleRange.PLUS_TO_MINUS_180;
        triggerAngle = 170;
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

    /**
     * The target angle is the angle that you think will be the final angle. If the target angle is
     * bigger than the trigger angle, then any angle read from the IMU will get adjusted to the range
     * determined by this target angle. The target angle is being used to automatically determine
     * the angle range. You could set it manually but then you have to know something about the
     * angle adjustement process. This is just an easy way to set it.
     * @param targetAngle
     */
    public void setTargetAngle(double targetAngle) {
        if (Math.abs(targetAngle) > Math.abs(triggerAngle)) {
            // the angles coming from the IMU will have to be adjusted since the anticipated target
            // or final angle is greater than the trigger angle
            if (targetAngle < 0) {
                // the anticipated target angle is negative so adjust all the angles so they
                // read 0 to -360
                angleRange = AngleRange.ZERO_TO_MINUS_360;
            }
            if (targetAngle > 0) {
                // the anticipated target angle is positive so adjust all the angles so they
                // read 0 to +360
                angleRange = AngleRange.ZERO_TO_PLUS_360;
            }
        } else {
            // the target angle is less than the threshold so angles get read as -180 to +180
            angleRange = AngleRange.PLUS_TO_MINUS_180;
        }
    }

    /**
     * Read the heading from the IMU and adjust it based on the angleRange so that it return an
     * angle in one of the following ranges:
     * -180 to +180
     * 0 to 360
     * -360 to 0
     * You will need to set your anticipated target angle before calling this method (setTargetAngle)
     * @return
     */
    public double getHeading() {
        return getAdjustedAngle(imu.getHeading());
    }

    /**
     * This method takes in an angle that ranges from -180 to +180 and then adjusts that angle to
     * one of 3 possible ranges: -180 to +180 (no change), 0 to -360, or 0 - +360.
     * Note that the desired angle range must be set before hand (setAngleRange). The adjustment
     * can be made when the input angle exceeds a certain trigger angle to keep the resulting
     * adjusted angle away from a transition point. See comments in code for more details.
     * Note that the trigger angle must be set before hand (setTriggerAngle)
     * @param angle angle is assumed to range grom -180 to +180
     * @return adjusted angle
     */
    public double getAdjustedAngle(double angle) {
        double adjustedAngle = 0;
        // check the desired mode (0 to +360, 0 to 360, -180 to 180)
        switch (angleRange) {
            case PLUS_TO_MINUS_180:
                // angle is already +180 to -180, don't do anything to it
                adjustedAngle = angle;
                break;
            case ZERO_TO_MINUS_360:
                // in -180 to +180, + is CCW (left half of clock face, - is CW (right half of clock face)
                // in this case the - angles on the right half of the clock face will remain as is
                // the + angles on the left half of the clock face will get translated to negative values
                // so 0 to + 180 become -360 to -180. So angle - 360 gives this. Examples:
                // +5 -> -355 (+5 - 360), this is about 11 o'clock
                // +175 -> -185 (+175 - 360), this is about 7 o'clock
                //        0                     -360/0
                //  +90       -90  becomes  -270     -90
                //    +180/-180                 -180
                // The translation only occurs after the trigger angle is exceeded. This feature can
                // be used to allow the robot to move away from the transition point before the angle
                // is adjusted. For example, start a turn to -175. Start the turn using -180 to 180
                // angles and only when the robot has moved to -90 does the angle get adjusted to
                //  0 to -360. That means the robot is well away from the -360/0 transition point
                // when the angles start to get adjusted.
                // Why is this needed? Well if the turn is started with adjusted angles and to robot
                // is right at the transition point, for example at angle = 0, the robot might cross
                // the transition point and the angle would suddenly read -360. The robot would reach
                // the desired angle, but it might go the long way around (-360 -> -270 -> -180 -> -90
                // instead of (0 -> -90).
                if (Math.abs(angle) > Math.abs(triggerAngle)) {
                    // if the input angle is positive, translate it to negative
                    if (angle > 0) {
                        adjustedAngle = angle - 360;
                    }
                }
                break;
            case ZERO_TO_PLUS_360:
                // in -180 to +180, + is CCW (left half of clock face, - is CW (right half of clock face)
                // in this case the + angles on the left half of the clock face will remain as is
                // the - angles on the right half of the clock face will get translated to positive values
                // so 0 to - 180 become -360 to -180. So angle - 360 gives this. Examples:
                // -5 -> +355 (-5 + 360), this is about 1 o'clock
                // -175 -> +185 (-175 + 360), this is about 5 o'clock
                //        0                     0/360
                //  +90       -90  becomes  +90      +270
                //    +180/-180                 +180
                // The translation only occurs after the trigger angle is exceeded. This feature can
                // be used to allow the robot to move away from the transition point before the angle
                // is adjusted. For example, start a turn to +175. Start the turn using -180 to 180
                // angles and only when the robot has moved to +90 does the angle get adjusted to
                //  0 to +360. That means the robot is well away from the 0/+360 transition point
                // when the angles start to get adjusted.
                if (Math.abs(angle) > Math.abs(triggerAngle))  {
                    // if the input angle is negative, translate it to positive
                    if (angle < 0) {
                        adjustedAngle = angle + 360;
                    }
                }
                break;
        }
        return adjustedAngle;
    }
}
