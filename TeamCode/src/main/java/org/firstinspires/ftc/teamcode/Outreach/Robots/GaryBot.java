package org.firstinspires.ftc.teamcode.Outreach.Robots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class GaryBot {

    public HardwareMap hwBot = null;

    //Drivetrain Motors
    public DcMotor frontLeftMotor;//1
    public DcMotor frontRightMotor;//0
    public DcMotor rearLeftMotor;//3
    public DcMotor rearRightMotor;//2

    // Fly Wheels and Variable Feeder Motors
    public DcMotorEx leftFlyWheel;
    public DcMotorEx rightFlyWheel;
    public DcMotor feederWheel;

    public LinearOpMode LinearOp = null;

    public static final double TICKS_PER_ROTATION = 386.3;
    public static final double ODO_TICKS_PER_ROTATION = 2000;


    // Helper method to set the run modes for all motors at the same
    public void setMotorRunModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }


    public GaryBot() {}

    //Init Method
    public void initRobot(HardwareMap hwMap) {
        hwBot = hwMap;

        //Drivetrain Motors HW Mapping
        frontLeftMotor = hwBot.dcMotor.get("front_left_motor");//Port 0 Control
        frontRightMotor = hwBot.dcMotor.get("front_right_motor");//Port 1 Control
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");//Port 2 Control
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");//Port 3 Control

        //Flywheels & Feed Wheel
        leftFlyWheel = hwBot.get(DcMotorEx.class, "left_fly_wheel");;//Port ex 0
        rightFlyWheel = hwBot.get(DcMotorEx.class, "right_fly_wheel");//Port ex 1
        feederWheel = hwBot.get(DcMotorEx.class,"feeder_wheel");//Port ex 2

        // Drivetrain Motor direction mapping
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Drivetrain Set Motor Run Modes
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drivetrain Motor break mapping
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Flywheel & Feeder Wheel Direction Mapping
        leftFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        feederWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Flywheel & Feed Wheel Breaking
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Flywheel & Feed Wheel Encoding for Using Velocity
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feederWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void flylaunch(double velocity ){

        leftFlyWheel.setVelocity(velocity);
        rightFlyWheel.setVelocity(velocity);

    }

    public void feedArtifact(double speed){

        feederWheel.setPower(speed);


    }



}
