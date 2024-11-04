package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Outreach.Robots.TankBot;


//@Disabled
@TeleOp(name = "Tank Differential Drive")

public class TankDifferentialDriveTeleOp extends OpMode {

    //TeleOp Driving Behavior Variables
    public double speedMultiply = 1;
    public enum Style {
        ARCADE1, ARCADE2, TANK
    }

    public Style driverStyle = Style.ARCADE1;

    // GamePad Variables
    public float leftStickY1;
    public float rightStickY1;
    public float leftStickX1;
    public float rightStickX1;

    public double leftMotorValue;
    public double rightMotorValue;

    // Construct the Physical Bot based on the Robot Class
    public TankBot Barry = new TankBot();


    // TeleOp Initialize Method.  This is the Init Button on the Driver Station Phone
    @Override
    public void init() {

        Barry.initRobot(hardwareMap);

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
        telemetryOutput();

    }

    public void telemetryOutput() {
        telemetry.addData("Drive Mode: ", driverStyle);
        telemetry.addData("Speed: ", speedMultiply);
        telemetry.addData("Front Left Motor Power: ", Barry.frontLeftMotor.getPower());
        telemetry.addData("Rear Left Motor Power: ", Barry.rearLeftMotor.getPower());
        telemetry.addData("Front Right Motor Power: ", Barry.frontRightMotor.getPower());
        telemetry.addData("Rear Right Motor Power: ", Barry.rearRightMotor.getPower());
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

                leftMotorValue = Math.pow(leftStickY1 - leftStickX1,3);
                rightMotorValue = Math.pow(leftStickY1 + leftStickX1,3);
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                Barry.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                Barry.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                Barry.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                Barry.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case ARCADE2:
                leftMotorValue = Math.pow(leftStickY1 - rightStickX1,3);
                rightMotorValue = Math.pow(leftStickY1 + rightStickX1,3);
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                Barry.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                Barry.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                Barry.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                Barry.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case TANK:
                double turningFactor = Math.abs(leftStickY1 - rightStickY1) * 0.5;  // Adjust factor as needed
                double powerFLM;
                double powerRLM;
                double powerFRM;
                double powerRRM;

                if (leftStickY1 > rightStickY1) {
                    // Turning left
                    powerFLM = leftStickY1 * speedMultiply * (1 - turningFactor);
                    powerRLM = leftStickY1 * speedMultiply * (1 - turningFactor);
                    powerFRM = rightStickY1 * speedMultiply * (1 + turningFactor);
                    powerRRM = rightStickY1 * speedMultiply * (1 + turningFactor);
                } else {
                    // Turning right
                    powerFLM = leftStickY1 * speedMultiply * (1 + turningFactor);
                    powerRLM = leftStickY1 * speedMultiply * (1 + turningFactor);
                    powerFRM = rightStickY1 * speedMultiply * (1 - turningFactor);
                    powerRRM = rightStickY1 * speedMultiply * (1 - turningFactor);
                }

                Barry.frontLeftMotor.setPower(powerFLM);
                Barry.rearLeftMotor.setPower(powerRLM);
                Barry.frontRightMotor.setPower(powerFRM);
                Barry.rearRightMotor.setPower(powerRRM);
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


}