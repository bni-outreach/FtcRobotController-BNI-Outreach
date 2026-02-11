package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Outreach.Robots.GaryBot;

@TeleOp(name = "Garry Teleop", group = "Drive")
public class GaryTeleOp extends OpMode {

      // Drivetrain Variables
    protected double leftStickY1;
    protected double leftStickX1;
    protected double rightStickX1;
    protected double rightStickY1;

    protected double leftMotorValue;
    protected double rightMotorValue;
    protected double powerThreshold;

    protected double speedMultiply = 1;

    // Drive Profile Control Variables
    protected  static final int PROFILE_1 = 1;  //User 1
    protected  static final int PROFILE_2 = 2; //user 2
    protected  int currentProfile = PROFILE_1;

    public double midGatePercent = 0.1;

    public enum Style {
        ARCADE1, ARCADE2, TANK, ANDY
    }

    public Style driverStyle = Style.ARCADE1;

    public double angularIncrease = 10;

    //Velocity of the Launching wheels
    protected double targetVelocity = 2000;
    protected double tolerance = 50;


    // Velocity Gates & Recovery Times for Near vs Far
    protected enum RangeMode { NEAR, FAR }
    protected RangeMode rangeMode = RangeMode.NEAR;


    // Instantiation of Robot using Robot Class Constructor
    public GaryBot bot = new GaryBot();


    @Override
    public void init() {
        bot.initRobot(hardwareMap);
        driverStyle = Style.ANDY;
    }

    public void input(){
        leftStickY1 = -gamepad1.left_stick_y;
        leftStickX1 = gamepad1.left_stick_x;
        rightStickY1 = gamepad1.right_stick_y;
        rightStickX1 = gamepad1.right_stick_x;
    }

    @Override
    public void loop() {
        flyWheelControl();
        feedWheelManualControl();
        input();
        driveControl();
        telemetryOutput();
    }



    // Robot Centric Drive Method
    public void driveControl() {
        if (gamepad1.a) {driverStyle = Style.ARCADE1;}
        if (gamepad1.b) {driverStyle = Style.ARCADE2;}
        if (gamepad1.y) {driverStyle = Style.TANK;}

        switch (driverStyle) {

            case ARCADE1:
                leftMotorValue = leftStickY1 - leftStickX1;
                rightMotorValue = leftStickY1 + leftStickX1;
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                bot.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                bot.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                bot.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                bot.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case ARCADE2:
                leftMotorValue = leftStickY1 - rightStickX1;
                rightMotorValue = leftStickY1 + rightStickX1;
                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);
                bot.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                bot.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                bot.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                bot.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                break;

            case TANK:
                double powerFLM = leftStickY1 * speedMultiply;
                double powerRLM = leftStickY1 * speedMultiply;
                double powerFRM = rightStickY1 * speedMultiply;
                double powerRRM = rightStickY1 * speedMultiply;

                bot.frontLeftMotor.setPower(powerFLM);
                bot.rearLeftMotor.setPower(powerRLM);
                bot.frontRightMotor.setPower(powerFRM);
                bot.rearRightMotor.setPower(powerRRM);
                break;

            case ANDY:
                leftMotorValue =  leftStickY1 + 2 * leftStickX1;
                rightMotorValue = leftStickY1 - 2 * leftStickX1;


                telemetry.addData("x", leftStickX1);
                telemetry.addData("y", leftStickY1);

                leftMotorValue = Range.clip(leftMotorValue, -1, 1);
                rightMotorValue = Range.clip(rightMotorValue, -1, 1);


                if(leftStickX1 > leftStickY1) {
                    bot.frontLeftMotor.setPower(leftMotorValue * speedMultiply * angularIncrease);
                    bot.rearRightMotor.setPower(rightMotorValue * speedMultiply * angularIncrease);
                    bot.rearLeftMotor.setPower(leftMotorValue * speedMultiply * angularIncrease);
                    bot.frontRightMotor.setPower(rightMotorValue * speedMultiply * angularIncrease);
                }
                else{
                    bot.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
                    bot.rearRightMotor.setPower(rightMotorValue * speedMultiply);
                    bot.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
                    bot.frontRightMotor.setPower(rightMotorValue * speedMultiply);
                }
                break;
        }
    }


    // ***** Manual Feeder Wheel Controller
    public void feedWheelManualControl() {
        if (gamepad1.left_trigger > 0.2) {
            bot.feedArtifact(1.0);
        }
        else if (gamepad1.right_trigger > 0.2) {
            bot.feedArtifact(-1.0);
        }
        else {
            bot.feedArtifact(0);
        }

    }

    // Fly Wheel Control
    public void flyWheelControl() {

        if (gamepad2.x) {       // Square
            targetVelocity = 1000;
        }
        if (gamepad2.a) {   // X
            // NEAR preset
            targetVelocity = 1500;  //781

        }
        if (gamepad2.b) { // Circle
            targetVelocity = 2000;

        }
        if (gamepad2.y) { // Triangle
            // FAR preset
            targetVelocity = 2500;

        }

        if (gamepad2.left_bumper) {
            targetVelocity = 0;
        }

        bot.flylaunch(targetVelocity);


        // Always command velocity each loop
        bot.leftFlyWheel.setVelocity(targetVelocity);
        bot.rightFlyWheel.setVelocity(targetVelocity);

    }

    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addLine("-------------------------------------");
        telemetry.update();
    }



    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }

}
