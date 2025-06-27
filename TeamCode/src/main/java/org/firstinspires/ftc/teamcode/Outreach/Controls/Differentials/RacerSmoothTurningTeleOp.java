package org.firstinspires.ftc.teamcode.Outreach.Controls.Differentials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Outreach.Robots.RacerBot;


//@Disabled
@TeleOp(name = "Racer:Differential:SmoothTurning")

public class RacerSmoothTurningTeleOp extends OpMode {

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
    public RacerBot Racer = new RacerBot();


    @Override
    public void init() {

        Racer.initRobot(hardwareMap);

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
        telemetry.addData("Front Left Motor Power: ", Racer.frontLeftMotor.getPower());
        telemetry.addData("Rear Left Motor Power: ", Racer.rearLeftMotor.getPower());
        telemetry.addData("Front Right Motor Power: ", Racer.frontRightMotor.getPower());
        telemetry.addData("Rear Right Motor Power: ", Racer.rearRightMotor.getPower());
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
        if (gamepad1.a) driverStyle = Style.ARCADE1;
        if (gamepad1.b) driverStyle = Style.ARCADE2;
        if (gamepad1.y) driverStyle = Style.TANK;

        double turningFactor = 0.5; // Adjust to control turn smoothness

        switch (driverStyle) {
            case ARCADE1:
                leftMotorValue = leftStickY1 - (leftStickX1 * turningFactor);
                rightMotorValue = leftStickY1 + (leftStickX1 * turningFactor);
                break;

            case ARCADE2:
                leftMotorValue = leftStickY1 - (rightStickX1 * turningFactor);
                rightMotorValue = leftStickY1 + (rightStickX1 * turningFactor);
                break;

            case TANK:
                if (leftStickY1 > rightStickY1) {
                    leftMotorValue = leftStickY1 * speedMultiply * (1 - turningFactor);
                    rightMotorValue = rightStickY1 * speedMultiply * (1 + turningFactor);
                } else {
                    leftMotorValue = leftStickY1 * speedMultiply * (1 + turningFactor);
                    rightMotorValue = rightStickY1 * speedMultiply * (1 - turningFactor);
                }
                break;
        }

        leftMotorValue = Range.clip(leftMotorValue, -1, 1);
        rightMotorValue = Range.clip(rightMotorValue, -1, 1);
        Racer.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
        Racer.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
        Racer.frontRightMotor.setPower(rightMotorValue * speedMultiply);
        Racer.rearRightMotor.setPower(rightMotorValue * speedMultiply);
    }


    public void speedControl () {
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


}
