package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Outreach.Robots.BigWheelBot;



//@Disabled
@TeleOp(name = "Big Wheels")

public class BigWheelTeleOp extends OpMode {

    //TeleOp Driving Behavior Variables
    public double speedMultiply = 1;
    public enum Style {
        ARCADE1, ARCADE2, TANK
    }

    public enum LoadStates {
        READY, LOAD, DELAY, UNLOAD
    }

    public LoadStates loadState = LoadStates.READY;
    public Style driverStyle = Style.ARCADE1;

    ElapsedTime timer = new ElapsedTime();

    // GamePad Variables
    public float leftStickY1;
    public float rightStickY1;
    public float leftStickX1;
    public float rightStickX1;

    public double leftMotorValue;
    public double rightMotorValue;

    // Construct the Physical Bot based on the Robot Class
    public BigWheelBot BigWheel = new BigWheelBot();


    // TeleOp Initialize Method.  This is the Init Button on the Driver Station Phone
    @Override
    public void init() {

        BigWheel.initRobot(hardwareMap);
        BigWheel.initFlyWheels(hardwareMap);
        BigWheel.initWormGears(hardwareMap);
        BigWheel.initServos(hardwareMap);

        leftStickY1 = 0;
        leftStickX1 = 0;
        rightStickY1 = 0;
        rightStickX1 = 0;

    }

    // TeleOp Loop Method.  This start AFTER clicking the Play Button on the Driver Station Phone

    public void loop() {

        getController();
        speedControl();
        driveControl();
        flyWheelControl();
        wormGearControl();
        servoControlManual();
        servoControlAutomatic();
        DiscLaunchControl();
        telemetryOutput();

    }

    public void telemetryOutput() {
        telemetry.addData("Drive Mode: ", driverStyle);
        telemetry.addData("Speed: ", speedMultiply);
        telemetry.addData("Front Left Motor Power: ", BigWheel.frontLeftMotor.getPower());
        telemetry.addData("Rear Left Motor Power: ", BigWheel.rearLeftMotor.getPower());
        telemetry.addData("Front Right Motor Power: ", BigWheel.frontRightMotor.getPower());
        telemetry.addData("Rear Right Motor Power: ", BigWheel.rearRightMotor.getPower());
        telemetry.update();

    }

    /**  ********  DRIVING METHODS USING GAMEPAD 1 *************      **/

    public void getController() {
        leftStickY1 = gamepad1.left_stick_y;
        leftStickX1 = gamepad1.left_stick_x;
        rightStickY1 = gamepad1.right_stick_y;
        rightStickX1 = gamepad1.right_stick_x;


    }

    public void driveControl() {

        if (gamepad1.a) {
            driverStyle = Style.ARCADE1;
        }
        if (gamepad1.b) {
            driverStyle = Style.ARCADE2;
        }
        if (gamepad1.y) {
            driverStyle = Style.TANK;
        }

        switch (driverStyle) {

            case ARCADE1:

                leftMotorValue = leftStickY1 - leftStickX1;
                rightMotorValue = leftStickY1 + leftStickX1;
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                BigWheel.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                BigWheel.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                BigWheel.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                BigWheel.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case ARCADE2:
                leftMotorValue = leftStickY1 - rightStickX1;
                rightMotorValue = leftStickY1 + rightStickX1;
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                BigWheel.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                BigWheel.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                BigWheel.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                BigWheel.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case TANK:

                double powerFLM = leftStickY1 * speedMultiply;
                double powerRLM = leftStickY1 * speedMultiply;
                double powerFRM = rightStickY1 * speedMultiply;
                double powerRRM = rightStickY1 * speedMultiply;

                BigWheel.frontLeftMotor.setPower(powerFLM);
                BigWheel.rearLeftMotor.setPower(powerRLM);
                BigWheel.frontRightMotor.setPower(powerFRM);
                BigWheel.rearRightMotor.setPower(powerRRM);
                break;
        }
    }



    public void speedControl () {
            if (gamepad1.dpad_right) {
                speedMultiply = 0.25;
            } else if (gamepad1.dpad_down) {
                speedMultiply = 0.50;
            } else if (gamepad1.dpad_left) {
                speedMultiply = 0.75;
            } else if (gamepad1.dpad_up) {
                speedMultiply = 1.00;
            }
    }

    public void flyWheelControl()
    {
        if (gamepad2.left_bumper) {
            BigWheel.rotateFlyWheel1(1.0);
            BigWheel.rotateFlyWheel2(-1.0);
        }

        if (gamepad2.right_bumper) {
            BigWheel.stopFlyWheel1();
            BigWheel.stopFlyWheel2();
        }

    }

    public void wormGearControl()
    {
        if (gamepad2.dpad_up ) {
            BigWheel.shooterTiltUp(0.90);
        }
        else if (gamepad2.dpad_down) {
            BigWheel.shooterTiltDown(-0.90);
        }
        else
        {
            BigWheel.setShooterTiltStop();
        }

        if (gamepad2.dpad_left) {
            BigWheel.shooterPanLeft(0.90);
        }
        else if (gamepad2.dpad_right) {
            BigWheel.shooterPanRight(-0.90);
        }
        else
        {
            BigWheel.setShooterPanStop();
        }

    }



    public void servoControlManual() {
        if (gamepad2.left_trigger > 0.1) {
            BigWheel.loadDiscFully();

        }

        if (gamepad2.right_trigger > 0.1) {
            BigWheel.unloadDisc();
        }
    }

    public void servoControlAutomatic() {
        if (gamepad2.y) {
            loadState = LoadStates.LOAD;

        }

    }

    public void DiscLaunchControl() {
        switch (loadState) {
            case LOAD:
                BigWheel.loadDiscFully();
                loadState = LoadStates.DELAY;
                timer.reset();
                break;
            case DELAY:
                if (timer.time() > .5) {
                    loadState = LoadStates.UNLOAD;
                }
                break;
            case UNLOAD:
                BigWheel.unloadDisc();
                loadState = LoadStates.READY;
                break;
            case READY:
                break;
        }
    }


}
