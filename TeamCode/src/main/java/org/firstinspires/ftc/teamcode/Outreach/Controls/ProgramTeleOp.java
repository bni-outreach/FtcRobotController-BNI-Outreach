package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Outreach.Robots.ProgramBot;

//@Disabled
@TeleOp (name = "ProgramBot TeleOp")
public class ProgramTeleOp extends OpMode {

    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold;
    double speedMultiply = 1.0;

    private static final int PROFILE_1 = 1;
    private static final int PROFILE_2 = 2;
    private int currentProfile = PROFILE_2;


    public ProgramBot BigHero6 = new ProgramBot();

    @Override
    public void init (){

        BigHero6.initRobot(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    @Override
    public void loop(){
        changeDriverProfile();
        speedControl();
        drive();
        telemetryOutput();
    }


    public void changeDriverProfile() {
        if (gamepad1.left_bumper) {
            currentProfile = PROFILE_1;
        }
        else if (gamepad1.right_bumper) {
            currentProfile = PROFILE_2;
        }

    }


    public void drive () {

        // Joystick values
        leftStickYVal = -gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);

        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        switch (currentProfile) {

            // Name of Driver using Profile 1
            case PROFILE_1:
                // leftStickXVal controls rotation, and rightStickXVal controls strafing.
                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
                break;
            // Name of Driver using Profile 2
            case PROFILE_2:
                //leftStickXVal controls strafing, and rightStickXVal controls rotation.
                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
                break;

            // Default Driver Profile
            default:
                frontLeftSpeed = 0;
                frontRightSpeed = 0;
                rearLeftSpeed = 0;
                rearRightSpeed = 0;
                break;
        }

        // Clipping motor speeds to [-1, 1]
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        // Setting motor powers (with threshold check)
        setMotorPower(BigHero6.frontLeftMotor, frontLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(BigHero6.frontRightMotor, frontRightSpeed, powerThreshold, speedMultiply);
        setMotorPower(BigHero6.rearLeftMotor, rearLeftSpeed, powerThreshold, speedMultiply);
        setMotorPower(BigHero6.rearRightMotor, rearRightSpeed, powerThreshold, speedMultiply);
    }

    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }

    public void telemetryOutput(){
        telemetry.addData("speed ", "Multipler ", + speedMultiply);
        telemetry.addData("pwr ", "FL motor ", + frontLeftSpeed);
        telemetry.addData("pwr ", "FR motor ", + frontRightSpeed);
        telemetry.addData("pwr ", "RL motor ", + rearLeftSpeed);
        telemetry.addData("pwr ", "RR motor ", + rearRightSpeed);
        telemetry.update();
    }

    public void speedControl(){
        if(gamepad1.dpad_up){
            speedMultiply = 1.0;
        }
        else if (gamepad1.dpad_right){
            speedMultiply = 0.75;
        }
        else if (gamepad1.dpad_down){
            speedMultiply = 0.50;
            }
        else if (gamepad1.dpad_left){
            speedMultiply = 0.25;
        }

    }


}