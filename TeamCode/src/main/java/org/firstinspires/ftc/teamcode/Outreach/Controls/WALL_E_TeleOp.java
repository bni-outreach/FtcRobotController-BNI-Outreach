package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Outreach.Robots.WalleBot;

//@Disabled
@TeleOp(name = "WALL-E")
public class WALL_E_TeleOp extends OpMode {


    //TeleOp Driving Behavior Variables
    public enum Style {
        ARCADE1, ARCADE2, TANK
    }
    public TankTeleOp.Style driverStyle = TankTeleOp.Style.ARCADE1;

    // GamePad Variables
    public float leftStickY1;
    public float rightStickY1;
    public float leftStickX1;
    public float rightStickX1;

    public double leftMotorValue;
    public double rightMotorValue;

    public double speedMultiply = 0.50;

    public double leftSidePower;
    public double rightSidePower;

    public double linearMotorPower = 0.85;
    boolean clawOpen = false;

    public double lazySusanPower = 0.90;

    public WalleBot WALL_E = new WalleBot();

    @Override
    public void init() {

        WALL_E.initRobot(hardwareMap);

        leftStickY1 = 0;
        leftStickX1 = 0;
        rightStickY1 = 0;
        rightStickX1 = 0;

    }

    public void loop() {
        getController();
        //driveControl();
        speedControl();
        driveControl();
        headControl();
        clawControl();
    }

    public void speedControl() {

        if (gamepad1.dpad_right) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 0.50;
        } else if (gamepad1.dpad_left) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_up) {
            speedMultiply = 1.00;
        }


    }

    public void getController() {
        leftStickY1 = gamepad1.left_stick_y;
        leftStickX1 = gamepad1.left_stick_x;
        rightStickY1 = gamepad1.right_stick_y;
        rightStickX1 = gamepad1.right_stick_x;


    }


    public void driverStyleControl() {

        if (gamepad1.a) {
            driverStyle = TankTeleOp.Style.ARCADE1;
        }
        if (gamepad1.b) {
            driverStyle = TankTeleOp.Style.ARCADE2;
        }
        if (gamepad1.y) {
            driverStyle = TankTeleOp.Style.TANK;
        }

    }

    public void driveControl() {

        switch (driverStyle) {

            case ARCADE1:

                leftMotorValue = leftStickY1 - leftStickX1;
                rightMotorValue = leftStickY1 + leftStickX1;
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                WALL_E.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                WALL_E.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                WALL_E.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                WALL_E.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case ARCADE2:
                leftMotorValue = leftStickY1 - rightStickX1;
                rightMotorValue = leftStickY1 + rightStickX1;
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                WALL_E.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                WALL_E.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                WALL_E.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                WALL_E.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case TANK:

                double powerFLM = leftStickY1 * speedMultiply;
                double powerRLM = leftStickY1 * speedMultiply;
                double powerFRM = rightStickY1 * speedMultiply;
                double powerRRM = rightStickY1 * speedMultiply;

                WALL_E.frontLeftMotor.setPower(powerFLM);
                WALL_E.rearLeftMotor.setPower(powerRLM);
                WALL_E.frontRightMotor.setPower(powerFRM);
                WALL_E.rearRightMotor.setPower(powerRRM);
                break;
        }

    }
        public void headControl() {
            if (gamepad1.right_stick_x < -0.1) {
                WALL_E.lazySusanLeft(lazySusanPower);
            } else if (gamepad1.right_stick_x > 0.1) {
                WALL_E.lazySusanRight(lazySusanPower);
            } else {
                WALL_E.lazySusanStop();
            }
        }


    public void clawControl() {

                if (gamepad1.right_trigger > 0.2) {
                    WALL_E.rightLinearActuatorForward(linearMotorPower);
                }
                if (gamepad1.right_bumper) {
                    WALL_E.rightLinearActuatorBack(linearMotorPower);
                } else {
                    WALL_E.rightLinearActuatorStop();
                }


                if (gamepad1.left_trigger > 0.2) {
                    WALL_E.leftLinearActuatorForward(linearMotorPower);
                } else if (gamepad1.left_bumper) {
                    WALL_E.leftLinearActuatorBack(linearMotorPower);
                } else {
                    WALL_E.leftLinearActuatorStop();
                }

                if (gamepad1.y) {
                    clawOpen = true;

                } else {
                    clawOpen = false;
                }
                if (clawOpen) {
                    WALL_E.leftClawOpen();
                } else if (clawOpen ) {
                    WALL_E.leftClawClose();
                }

    }





    }


