package org.firstinspires.ftc.teamcode.Outreach.Robots;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Outreach.Drivetrains.TankFourMotorDrive;

public class BigWheelBot extends TankFourMotorDrive {

    // Motors for Mechanisms
    public DcMotor shooterPan;
    public DcMotor shooterTilt;
    public DcMotor flyWheel1;
    public DcMotor flyWheel2;
    public VoltageSensor voltageSensor;

    // Servos for Mechanisms
    public Servo discLoader;

    // Limelight
    public Limelight3A cam = null;

    // Hardware Mapping Variable used by robot controller
    public HardwareMap hwBot = null;

    // Robot Physical Constructor used in TeleOp and Autonomous classes
    public BigWheelBot() { }

    // Custom Method that will initialize the robot hardware in TeleOp and Autonomous
    public void initRobot (HardwareMap hwMap) {

        hwBot = hwMap;

        //Define the name of the motors used in the control hub configuration
        frontLeftMotor =  hwBot.dcMotor.get("front_left_motor");    // Control Hub Port 0
        rearLeftMotor =  hwBot.dcMotor.get("rear_left_motor");     // Control Hub Port 1
        frontRightMotor = hwBot.dcMotor.get("front_right_motor");  // Control Hub Port 2
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");    // Control Hub Port 3

        //Sets the direction of the robot's motors based on physical placement
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Define this robot run modes
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define this robot's braking modes
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // **** Initialize Limelight Camera ****
    public void initLimelight(HardwareMap hwMap) {
        hwBot = hwMap;
        cam = hwBot.get(Limelight3A.class, "limelight");
        cam.setPollRateHz(100);
    }

    // **** Initialize Fly Wheel Hardware ****
    public void initFlyWheels(HardwareMap hwMap) {
        hwBot = hwMap;
        flyWheel1 = hwBot.dcMotor.get("fly_wheel1");                        //Expansion Port 2
        flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheel2 = hwBot.dcMotor.get("fly_wheel2");                        //Expansion Port 3
        flyWheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void initVoltageSensor(HardwareMap hwMap) {
        hwBot = hwMap;
        voltageSensor = hwBot.voltageSensor.iterator().next();
    }

    // **** Initialize Worm Gear Mechanism ****
    public void initWormGears(HardwareMap hwMap) {
        hwBot = hwMap;
        shooterPan = hwBot.dcMotor.get("shooter_pan");                          //Expansion Port 0
        shooterPan.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterPan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterTilt = hwBot.dcMotor.get("shooter_tilt");                          //Expansion Port 1
        shooterTilt.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // **** Initialize Servo  Hardware ****
    public void initServos(HardwareMap hwMap) {
        hwBot = hwMap;
        discLoader = hwBot.get(Servo.class, "disc_loader");          //Control Hub Servo Port 0
        discLoader.setDirection(Servo.Direction.FORWARD);
    }

    // **** Movement Methods for Fly Wheels ****
    public void rotateFlyWheel1 (double power) {
        flyWheel1.setPower(power);
    }

    public void rotateFlyWheel2 (double power) {
        flyWheel2.setPower(power);
    }

    public void stopFlyWheel1 () {
        flyWheel1.setPower(0);
    }

    public void stopFlyWheel2 () {
        flyWheel2.setPower(0);
    }

    // **** Movement Methods for Panning ****
    public void shooterPanRight (double power) {

        shooterPan.setPower(Math.abs(power));
    }

    public void shooterPanLeft (double power) {

        shooterPan.setPower(-Math.abs(power));
    }

    public void setShooterPanStop () {

        shooterPan.setPower(0);
    }

    // **** Movement Methods for Tilting ****
    public void shooterTiltUp (double power) {

        shooterTilt.setPower(Math.abs(power));
    }

    public void shooterTiltDown (double power) {

        shooterTilt.setPower(-Math.abs(power));
    }

    public void setShooterTiltStop () {

        shooterTilt.setPower(0);
    }


    // **** Movement Methods for Disc Loader ****
    public void loadDiscFully() {
        discLoader.setPosition(0.6);
    }
    public void loadDiscPartially() {

        discLoader.setPosition(0.4);
    }
    public void unloadDisc() {

        discLoader.setPosition(0.3);
    }


}





