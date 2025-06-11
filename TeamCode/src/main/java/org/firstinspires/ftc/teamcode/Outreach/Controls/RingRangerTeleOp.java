package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Outreach.Robots.RingRangerBot;


@TeleOp(name = "Ringed Ranger")
//@Disabled
public class RingRangerTeleOp extends OpMode {

    // Variables & Constants specific to TeleLabBot
    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;
    double LauncherSpeed;

    double launcherPower = 1;
    double launcherVelocity = 1400; //1370 before
    double PowerShotVelocity = 1235; //1255 reduced by 20

    boolean PushToggle = false;
    boolean PushToggleMag = false;


    double powerThreshold = 0;
    double speedMultiply = 1;

    boolean reverseModeToggle = false;
    boolean prankToggle = false;
    boolean rapidFireEngage = false;
    boolean rapidFireEngage2 = false;

    public RingRangerBot Bot = new RingRangerBot();


    @Override
    public void init() {
        Bot.initRobot(hardwareMap, "TeleOp", "TeleOp");

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        Bot.WobbleClosed();
        Bot.wobbleArmRaiseEngage = false;
        Bot.wobbleArmLowerengage = false;
        Bot.RingMagDown();
        Bot.RingPush();

    }

    @Override
    public void loop() {
        drive();
        wobble();
        launcher();
        intake();
        driveMode();
        telemetryOutput();
    }


    @Override
    public void stop() {

    }

    public void driveMode() {
        if (gamepad1.left_bumper) {
            speedMultiply = 0.3;
        } else if (gamepad1.right_bumper) {
            speedMultiply = 1;
        }


        if (gamepad1.dpad_right) {
            reverseModeToggle = true;
        }
        if (gamepad1.dpad_left) {
            reverseModeToggle = false;
        }

    }

    public void drive() {
/*
        leftStickYVal = -gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

        frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

        rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

        rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);


        if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
            frontLeftSpeed = 0;
            Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
        } else {
            Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
        }

        if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold){
            frontRightSpeed = 0;
            Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
        } else {
            Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
        }

        if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
            rearLeftSpeed = 0;
            Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
        } else {
            Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
        }

        if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold){
            rearRightSpeed = 0;
            Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
        } else {
            Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
        }

        telemetryOutput();*/


        if (reverseModeToggle) {

            leftStickYVal = --gamepad1.left_stick_y;
            leftStickYVal = Range.clip(leftStickYVal, -1, 1);
            leftStickXVal = gamepad1.left_stick_x;
            leftStickXVal = Range.clip(leftStickXVal, -1, 1);
            rightStickXVal = -gamepad1.right_stick_x;
            rightStickXVal = Range.clip(rightStickXVal, -1, 1);

            Bot.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            Bot.rearLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            Bot.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            Bot.rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

            frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

            rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
            rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

            rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
            rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

            if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                frontLeftSpeed = 0;
                Bot.frontLeftMotor.setPower(frontLeftSpeed);
            } else {
                Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
            }

            if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
                frontRightSpeed = 0;
                Bot.frontRightMotor.setPower(frontRightSpeed);
            } else {
                Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
            }

            if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                rearLeftSpeed = 0;
                Bot.rearLeftMotor.setPower(rearLeftSpeed);
            } else {
                Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
            }

            if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
                rearRightSpeed = 0;
                Bot.rearRightMotor.setPower(rearRightSpeed);
            } else {
                Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
            }
        }
        else if (prankToggle){
            leftStickYVal = -gamepad1.left_stick_y;
            leftStickYVal = Range.clip(leftStickYVal, -1, 1);
            leftStickXVal = gamepad1.left_stick_x;
            leftStickXVal = Range.clip(leftStickXVal, -1, 1);
            rightStickXVal = gamepad1.right_stick_x;
            rightStickXVal = Range.clip(rightStickXVal, -1, 1);
            Bot.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            Bot.rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            Bot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            Bot.rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

            frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

            rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
            rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

            rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
            rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

            if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                frontLeftSpeed = 0;
                Bot.frontLeftMotor.setPower(frontLeftSpeed);
            } else {
                Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
            }

            if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
                frontRightSpeed = 0;
                Bot.frontRightMotor.setPower(frontRightSpeed);
            } else {
                Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
            }

            if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                rearLeftSpeed = 0;
                Bot.rearLeftMotor.setPower(rearLeftSpeed);
            } else {
                Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
            }

            if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
                rearRightSpeed = 0;
                Bot.rearRightMotor.setPower(rearRightSpeed);
            } else {
                Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
            }
        }
        else {

            leftStickYVal = -gamepad1.left_stick_y;
            leftStickYVal = Range.clip(leftStickYVal, -1, 1);
            leftStickXVal = gamepad1.left_stick_x;
            leftStickXVal = Range.clip(leftStickXVal, -1, 1);
            rightStickXVal = gamepad1.right_stick_x;
            rightStickXVal = Range.clip(rightStickXVal, -1, 1);
            Bot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            Bot.rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            Bot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            Bot.rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

            frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

            rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
            rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

            rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
            rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

            if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                frontLeftSpeed = 0;
                Bot.frontLeftMotor.setPower(frontLeftSpeed);
            } else {
                Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
            }

            if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
                frontRightSpeed = 0;
                Bot.frontRightMotor.setPower(frontRightSpeed);
            } else {
                Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
            }

            if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                rearLeftSpeed = 0;
                Bot.rearLeftMotor.setPower(rearLeftSpeed);
            } else {
                Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
            }

            if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
                rearRightSpeed = 0;
                Bot.rearRightMotor.setPower(rearRightSpeed);
            } else {
                Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
            }
        }
    }


    public void wobble() {

        if (gamepad2.dpad_up == true) {
            Bot.WobbleArmRaised(1.0);
        } else if (gamepad2.dpad_down == true) {
            Bot.WobbleArmLower(1.0);
        } else {
            if (Bot.wobbleArmRaiseEngage == false && Bot.wobbleArmLowerengage == false) {
                Bot.WobbleArmStopMotors();
            }
        }


// Servo Grabber controls
        if (gamepad2.right_bumper == true) {
            Bot.WobbleOpen();
        }
        if (gamepad2.left_bumper == true) {
            Bot.WobbleClosed();
        }

    }

    public void launcher() {
        if (gamepad2.y == true && PushToggleMag == false) {
            if (Bot.ringIncrement <= 3) {
                Bot.LauncherOn(launcherVelocity);
            }
            else {
                Bot.LauncherOn(launcherVelocity-20);
            }

//            Bot.RingMagUp();
            PushToggleMag = true;
//            Bot.launcherMotor2.setV
        }

        if (PushToggleMag == true) {
            Bot.RingMagIncrement();
        }

        if (gamepad2.a == true) {
            Bot.LauncherOff(0);
//            motor_left.setPower(0);
//            motor_right.setPower(0);
            Bot.RingMagDown();
        }
        if (Bot.RingMag.getPosition() >= Bot.RingMagUpPos) {
            PushToggleMag = false;
        }

        if (gamepad2.x == true) {
            Bot.rapidFireRing = 0;
            rapidFireEngage = true;
        }
        if (rapidFireEngage == true) {
            if (Bot.rapidFireRing == 5) {
                rapidFireEngage = false;
                Bot.RingPush();
            } else {
                Bot.rapidFire();


            }
        }
        if (gamepad2.right_trigger > 0.1){
            Bot.LauncherOn(PowerShotVelocity);
            Bot.RingMagUp();
        }
        if (gamepad2.dpad_left == true){
            Bot.LauncherOn(PowerShotVelocity);
        }
        if (gamepad2.dpad_right == true){
            Bot.LauncherOn(launcherVelocity);
        }

        if (gamepad1.a == true) {
            Bot.LauncherOn(PowerShotVelocity);
//            launcherVelocity -= 10;
//            Bot.LauncherOn(launcherVelocity);
            }
        if (gamepad1.y == true) {
            Bot.LauncherOn(launcherVelocity);
//            launcherVelocity += 10;
//            Bot.LauncherOn(launcherVelocity);
            }
        if (gamepad2.b == true){
            Bot.rapidFireRing = 0;
            rapidFireEngage2 = true;
            }
        if (rapidFireEngage2 == true) {
            if (Bot.rapidFireRing == 3) {
                rapidFireEngage2 = false;
                Bot.RingPush();
            } else {
                Bot.rapidFire();
                }
        }
    }
        public void intake () {
            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                Bot.IntakeOn(gamepad2.left_stick_y);
                Bot.SpinInIntakeCorrector();
            }
            else {
                Bot.IntakeOff(0);
                Bot.StopIntakeCorrector();
            }
        }

        public void telemetryOutput() {
            telemetry.addLine(String.format("Voltage: %.1f", Bot.voltageSensor.getVoltage()));
            telemetry.addData("Launcher Coeeficnent", Bot.launchCoefficient);
            telemetry.addData("Target Velocity", launcherPower * Bot.launchCoefficient);
            telemetry.addData("1 motor power: ", Bot.launcherMotor1.getPower());

        telemetry.addData("2 motor power: ", Bot.launcherMotor2.getPower());
        telemetry.addData("1 motor velocity: ", Bot.launcherMotor1.getVelocity());
        telemetry.addData("2 motor velocity: ", Bot.launcherMotor2.getVelocity());
        telemetry.addData("Velocity: ", launcherVelocity);
        telemetry.addData("Rapid Fire Ring: ", Bot.rapidFireRing);
        telemetry.addData("rapidFireEngage", rapidFireEngage);
        telemetry.addData("rapidFireEngage2", rapidFireEngage2);
        telemetry.addData("Servo continious", Bot.IntakeCorrector.getPower());
        telemetry.update();
        }

}
