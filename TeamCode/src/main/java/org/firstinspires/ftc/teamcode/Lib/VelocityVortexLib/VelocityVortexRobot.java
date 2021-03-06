package org.firstinspires.ftc.teamcode.Lib.VelocityVortexLib;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Lib.FTCLib.DriveTrain;

public class VelocityVortexRobot {

    //*********************************************************************************************
    //          ENUMERATED TYPES
    //
    // user defined types
    //
    //*********************************************************************************************

    private enum RobotMode {
        AUTONOMOUS,
        TELEOP
    }

    //*********************************************************************************************
    //          PRIVATE DATA FIELDS
    //
    // can be accessed only by this class, or by using the public
    // getter and setter methods
    //*********************************************************************************************

    // Here are all of the objects that make up the entire robot
    // note that the IMU is an object in the drive train
    public RobotMode robotMode;
    public DriveTrain driveTrain;
    //public FrontBeaconPusher frontBeaconPusher;
    public MuxPlusColorSensors muxPlusColorSensors;
    public VelocityVortexSweeper sweeper;
    public VelocityVortexShooter shooter;
    public SideBeaconPusher rightSideBeaconPusher;
    public SideBeaconPusher leftSideBeaconPusher;
    public FrontBeaconPusher frontBeaconPusher;
    public FrontBeaconPusherControl frontBeaconPusherControl;
    public AllianceColorSwitch allianceColorSwitch;
    public AllianceColorSwitch.AllianceColor allianceColor;
    // public FrontBeaconPusher frontBeaconPusher;
    // public I2CMux mux;
    // public BallShooter ballShooter;

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

    private VelocityVortexRobot(HardwareMap hardwareMap, RobotMode robotMode, Telemetry telemetry) {
        if(robotMode == RobotMode.AUTONOMOUS) {
            driveTrain = DriveTrain.DriveTrainAutonomous(hardwareMap, telemetry);
            telemetry.addData("Drive Train Initialized", "!");
            telemetry.update();
            allianceColorSwitch = new AllianceColorSwitch(hardwareMap, telemetry);
            telemetry.addData("Drive Train Initialized", "!");
            telemetry.addData("Color Switches Initialized", "!");
            telemetry.update();
            allianceColor = allianceColorSwitch.getAllianceColor();
        } else {
            driveTrain = DriveTrain.DriveTrainTeleOp(hardwareMap, telemetry);
            allianceColor = AllianceColorSwitch.AllianceColor.BLUE;
        }
        sweeper = new VelocityVortexSweeper(hardwareMap);
        telemetry.addData("Drive Train Initialized", "!");
        telemetry.addData("Color Switches Initialized", "!");
        telemetry.addData("Sweeper Initialized", "!");
        telemetry.update();
        muxPlusColorSensors = new MuxPlusColorSensors(hardwareMap, telemetry);
        telemetry.addData("Drive Train Initialized", "!");
        telemetry.addData("Color Switches Initialized", "!");
        telemetry.addData("Sweeper Initialized", "!");
        telemetry.addData("MuxPlusColorSensors Initialized", "!");
        telemetry.update();
        rightSideBeaconPusher = new SideBeaconPusher(hardwareMap, telemetry, driveTrain, SideBeaconPusher.SideBeaconPusherPosition.RIGHT, muxPlusColorSensors);
        telemetry.addData("Drive Train Initialized", "!");
        telemetry.addData("Color Switches Initialized", "!");
        telemetry.addData("Sweeper Initialized", "!");
        telemetry.addData("MuxPlusColorSensors Initialized", "!");
        telemetry.addData("Right side beacon pusher Initialized", "!");
        telemetry.update();
        //frontBeaconPusher = new FrontBeaconPusher(hardwareMap, telemetry, muxPlusColorSensors);
        frontBeaconPusherControl = new FrontBeaconPusherControl(hardwareMap, telemetry, muxPlusColorSensors, allianceColor, driveTrain);
        telemetry.addData("Drive Train Initialized", "!");
        telemetry.addData("Color Switches Initialized", "!");
        telemetry.addData("Sweeper Initialized", "!");
        telemetry.addData("MuxPlusColorSensors Initialized", "!");
        telemetry.addData("Rigght side beacon pusher Initialized", "!");
        telemetry.addData("Front beacon pusher Control Initialized", "!");
        telemetry.update();
        shooter = new VelocityVortexShooter(hardwareMap, telemetry);
        telemetry.addData("Drive Train Initialized", "!");
        telemetry.addData("Color Switches Initialized", "!");
        telemetry.addData("Sweeper Initialized", "!");
        telemetry.addData("MuxPlusColorSensors Initialized", "!");
        telemetry.addData("Rigght side beacon pusher Initialized", "!");
        telemetry.addData("Front beacon pusher Initialized", "!");
        telemetry.addData("Shooter initialized", "!");
        telemetry.update();
        //frontBeaconPusher = new FrontBeaconPusher(hardwareMap, telemetry, muxPlusColorSensors);
        init();
    }


    public static VelocityVortexRobot createRobotForAutonomous(HardwareMap hardwareMap, Telemetry telemetry) {
        VelocityVortexRobot robot = new VelocityVortexRobot(hardwareMap, RobotMode.AUTONOMOUS, telemetry);
        return robot;
    }

    public static VelocityVortexRobot createRobotForTeleop(HardwareMap hardwareMap, Telemetry telemetry) {
        VelocityVortexRobot robot = new VelocityVortexRobot(hardwareMap, RobotMode.TELEOP, telemetry);
        return robot;
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

    public void init() {
        sweeper.init();
        shooter.init();
        rightSideBeaconPusher.init();
    }

    public void update() {
        sweeper.update();
        shooter.update();
        frontBeaconPusher.updateState();

    }

    public void shutdown() {
        sweeper.shudown();
        driveTrain.shutdown();
        shooter.shutdown();
        rightSideBeaconPusher.shutdown();
    }

    // most of the functionality of the robot is reached by calling methods in the objects that make
    // up the robot. For example:
    // rightSideBeaconPusher.pushBeacon()
    // ballShooter.shoot()
}
