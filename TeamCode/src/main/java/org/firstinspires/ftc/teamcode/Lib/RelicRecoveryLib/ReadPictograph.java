package org.firstinspires.ftc.teamcode.Lib.RelicRecoveryLib;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class ReadPictograph {

    //*********************************************************************************************
    //          PRIVATE DATA FIELDS
    //
    // can be accessed only by this class, or by using the public
    // getter and setter methods
    //*********************************************************************************************
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;

    private VuforiaLocalizer vuforia;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    private int cameraMonitorViewId;

    private VuforiaLocalizer.Parameters parameters;
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
    public ReadPictograph(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        init(hardwareMap);
        // MATT and JARED - this method is not supposed to run until after play is pressed - so
        // I commented it out
        //runAtStart();
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
    public void runAtStart() {
        relicTrackables.activate();
    }

    public void init(HardwareMap hardwareMap) {
        this.cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdFJpV3/////AAAAGfTNpwzHLEWLhTazSUptJDdvoaO1q58UH3Ix0gMjIizeGqeRTy/mHyHpZI3hX3VrQk0S4VJKsiwBIUrTZy57oWoQQGsD/6JXnrC/R2zQ2ruhxmV9JYc6zr5Lhu+aUFdce/WJezBkcUv7fD2y6kmNHAWlYyMx3ZP8YX2bSfTWu4PjiO3N/CFelgIJSz5BCRtYeFb1gKkCYhsqKUNfkWXznEFvX8ppW72yjbfq62QwqGFeuql/3cPce8asiOVo9NLiG9mIuADM+FWLairEHQ4h2euGHa+JNrk36EO0zVAFk9G2RBQJRkwgA7jUOGpCEW0Qqt0XJoM+0T0sOVORrn3Lqp9M4ecaQYsQsR8RPoZRL0Ox";

        this.parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        this.relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        this.relicTemplate = relicTrackables.get(0);
        this.relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public RelicRecoveryVuMark getvuMark() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }
}