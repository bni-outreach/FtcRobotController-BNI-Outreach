package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Outreach.Robots.GaryBot;
import java.util.ArrayList;
import java.util.List;

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

    protected double speedMultiply = 0.75;

    // Drive Profile Control Variables
    protected  static final int PROFILE_1 = 1;  //User 1
    protected  static final int PROFILE_2 = 2; //user 2
    protected  int currentProfile = PROFILE_1;

    public enum Style {
        ARCADE1, ARCADE2, TANK
    }

    public Style driverStyle = Style.ARCADE1;

    protected boolean isIntaking = false;
    protected boolean isTracking = false;

    //Velocity of the Launching wheels
    protected double targetVelocity;
    protected double tolerance = 50;

    protected double min_velocity_drop = 50; // threshold for detecting ball contact
    protected List<Double> previousShotVelocityL = new ArrayList<>();

    protected boolean hasStartedAutoLaunch = false;
    protected boolean hasStartedFeeding = false;
    protected boolean hasReleased = true;

    protected double transferServoSpeed = 1;
    protected double intakeMotorSpeed = 1;

    // Instantiation of Robot using Robot Class Constructor
    public GaryBot bot = new GaryBot();


    //Shooting variables
    protected boolean isLaunching = false;


    @Override
    public void init() {
        bot.initRobot(hardwareMap);
    }

    @Override
    public void loop() {
        driverOneInput();
        driverTwoInput();
        setFlywheelSpeed();
        robotCentricDrive();
        telemetryOutput();
    }



    // Robot Centric Drive Method
    public void robotCentricDrive() {
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
        }
    }



    //********* Firing Control ****************************
    public boolean canLaunch(double selected_speed, double tolerance) {
        double upper_tolerance = Math.max(0, selected_speed + tolerance);
        double lower_tolerance = Math.max(0, selected_speed - tolerance);
        double currentVelocity = getCurrentVelocity();

        return currentVelocity >= lower_tolerance && currentVelocity <= upper_tolerance;
    }

    public void setFlywheelSpeed(){
        //Launch if is allowed to
        bot.flylaunch(isLaunching ? targetVelocity : 0);
    }
    // ***** Manual Feeder Wheel Controller
    public void driverOneInput() {

        //Change ground move speed
        if (gamepad1.dpad_right) {
            speedMultiply = 0.5;
        } else if (gamepad1.dpad_left) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_up) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 1;
        }

        if(gamepad1.left_bumper){
            isIntaking = true;
        }
        if(gamepad1.right_bumper){
            isIntaking = false;
        }

        if(isIntaking){
            bot.intakeControl(gamepad1.left_trigger > 0.5 ? -intakeMotorSpeed : intakeMotorSpeed);
        }
        else{
            bot.intakeControl(0);
        }
    }

    public void driverTwoInput(){

        //Control transfer servo
        if(gamepad1.right_bumper){
            bot.transferSpeedCon(transferServoSpeed);
        }
        else if(gamepad1.left_bumper){
            bot.transferSpeedCon(-transferServoSpeed);
        }
        else{
            bot.transferSpeedCon(0);
        }

        if (gamepad1.x) { // Square
            targetVelocity = 1000;
        }
        if (gamepad1.a) { // X
            targetVelocity = 1500;
        }
        if (gamepad1.b) { // Circle
            targetVelocity = 2000;
        }
        if (gamepad1.y) { // Triangle
            targetVelocity = 2500;
        }
        if(gamepad1.right_stick_button)
        {
            targetVelocity = 3000;
        }


        if (gamepad1.left_trigger > 0.5){
            isLaunching = false;
        }
        if (gamepad1.right_trigger > 0.5){
            isLaunching = true;
        }

    }


    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addLine("-------------------------------------");
        telemetry.update();
    }

    //******** Helper Functions **************************
    private double getCurrentVelocity() {
        // Averages front and back motor velocity
        return (bot.launchFrontMotor.getVelocity() + bot.launchBackMotor.getVelocity()) / 2.0;
    }

    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }

}
