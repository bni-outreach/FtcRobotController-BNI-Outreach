package org.firstinspires.ftc.teamcode.Outreach.Controls;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

    public Style driverStyle = Style.ARCADE1;

    public enum LoadStates {
        READY, LOAD, DELAY, UNLOAD
    }

    public LoadStates loadState = LoadStates.READY;


    public enum D2Style {
        Manual, Auto
    }

    public D2Style d2Style = D2Style.Manual;

    ElapsedTime timer = new ElapsedTime();


    // GamePad Variables
    public float leftStickY1;
    public float rightStickY1;
    public float leftStickX1;
    public float rightStickX1;

    public double leftMotorValue;
    public double rightMotorValue;

    //Limelight Variables
    public double tXErrorMultiplier = .02;
    public double tYErrorMultiplier = .015;
    public double errorOffset = 7;
    public double minimumCommand = 0.1;
    public double maximumCommand = 0.4;

    //Limelight Results
    private LLResult result = null;

    // For toggling D2Style with gamepad2.a
    private boolean prevD2A = false;


    // Construct the Physical Bot based on the Robot Class
    public BigWheelBot BigWheel = new BigWheelBot();


    // TeleOp Initialize Method.  This is the Init Button on the Driver Station Phone
    @Override
    public void init() {

        BigWheel.initRobot(hardwareMap);
        BigWheel.initFlyWheels(hardwareMap);
        BigWheel.initVoltageSensor(hardwareMap);
        BigWheel.initWormGears(hardwareMap);
        BigWheel.initServos(hardwareMap);
        BigWheel.initLimelight(hardwareMap);

        leftStickY1 = 0;
        leftStickX1 = 0;
        rightStickY1 = 0;
        rightStickX1 = 0;

    }

    @Override
    public void start() {
        BigWheel.cam.start();
        BigWheel.cam.pipelineSwitch(0);

    }


    @Override
    public void loop() {

        getController();

        // Toggle D2Style with gamepad2.a
        // This runs only on the rising edge: when button is just pressed
        if (gamepad2.a && !prevD2A) {
            if (d2Style == D2Style.Manual) {
                d2Style = D2Style.Auto;
            } else {
                d2Style = D2Style.Manual;
            }
        }
        prevD2A = gamepad2.a;

        speedControl();
        driveControl();

        if (d2Style == D2Style.Manual) {
            flyWheelControl();
            wormGearControl();
            servoControlManual();
            servoControlAutomatic();
        } else if (d2Style == D2Style.Auto) {
            //autoTurret();
            autoTurretWithVisionModel();
        }

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

    /**
     * *******  DRIVING METHODS USING GAMEPAD 1 *************
     **/

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


    public void speedControl() {
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

    public void flyWheelControl() {
        double nominalVoltage = 12.0;
        double currentVoltage = BigWheel.voltageSensor.getVoltage();
        double compensatedPower = nominalVoltage / currentVoltage;
        compensatedPower = Math.min(compensatedPower, 1.0);  // Cap at 100% power

        if (gamepad2.left_bumper) {
            BigWheel.rotateFlyWheel1(compensatedPower);
            BigWheel.rotateFlyWheel2(-compensatedPower);
        }

        if (gamepad2.right_bumper) {
            BigWheel.stopFlyWheel1();
            BigWheel.stopFlyWheel2();
        }
        telemetry.addData("Battery Voltage", currentVoltage);
        telemetry.addData("Flywheel Power", compensatedPower);

    }

    public void wormGearControl() {
        if (gamepad2.dpad_up) {
            BigWheel.shooterTiltUp(0.90);
        } else if (gamepad2.dpad_down) {
            BigWheel.shooterTiltDown(-0.90);
        } else {
            BigWheel.setShooterTiltStop();
        }

        if (gamepad2.dpad_left) {
            BigWheel.shooterPanLeft(0.90);
        } else if (gamepad2.dpad_right) {
            BigWheel.shooterPanRight(-0.90);
        } else {
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


    public void getCamResult() {
        result = BigWheel.cam.getLatestResult();
    }

    public boolean lastResultValid() {
        return result != null && result.isValid();
    }

    public void autoTurret() {

        double nominalVoltage = 12.0;
        double currentVoltage = BigWheel.voltageSensor.getVoltage();
        double compensatedPower = nominalVoltage / currentVoltage;
        compensatedPower = Math.min(compensatedPower, 1.0);  // Cap at 100% power

        BigWheel.unloadDisc();
        BigWheel.rotateFlyWheel1(compensatedPower);
        BigWheel.rotateFlyWheel2(-compensatedPower);

        getCamResult();
        if (lastResultValid()) {
            double tx = result.getTx();  // Horizontal offset from center (-27 to 27 deg)
            double ty = result.getTy();  // Vertical offset from center
            double ta = result.getTa();  // Target area (size) as % of image

            // Add deadband around errorOffset and correct pan direction
            if (tx > errorOffset + 0.01) {
                BigWheel.shooterPanRight(tx * tXErrorMultiplier);
                telemetry.addLine("Pan Right: " + tx);
            } else if (tx < -(errorOffset + 0.01)) {
                BigWheel.shooterPanLeft(tx * tXErrorMultiplier);
                telemetry.addLine("Pan Left: " + tx);
            } else {
                BigWheel.setShooterPanStop();
                telemetry.addLine("Pan Stop");
                try {
                    Thread.sleep(50);  // Small delay to ensure motor stops
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            // Add deadband around errorOffset and correct tilt direction
            if (ty > errorOffset + 0.01) {
                BigWheel.shooterTiltUp(ty * tYErrorMultiplier);
                telemetry.addLine("Tilt Up: " + ty);
            } else if (ty < -(errorOffset + 0.01)) {
                BigWheel.shooterTiltDown(ty * tYErrorMultiplier);
                telemetry.addLine("Tilt Down: " + ty);
            } else {
                BigWheel.setShooterTiltStop();
                telemetry.addLine("Tilt Stop");
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            if (ta > 10 && ta < 40 && Math.abs(tx) < errorOffset && Math.abs(ty) < errorOffset) {
                BigWheel.loadDiscFully();
                telemetry.addLine("Load");
                d2Style = D2Style.Manual;
            }

            // Add debug telemetry
            telemetry.addData("TX", tx);
            telemetry.addData("TY", ty);
            telemetry.addData("TA", ta);
            telemetry.addData("Pan Power", BigWheel.shooterPan.getPower());
            telemetry.addData("Tilt Power", BigWheel.shooterTilt.getPower());
            telemetry.addData("Battery Voltage", currentVoltage);
            telemetry.addData("Flywheel Power", compensatedPower);
        }
    }

    public void autoTurretWithVisionModel() {
        double confidenceThreshold = 0.7;

        LLResult visionResult = BigWheel.cam.getLatestResult();

        if (visionResult != null && visionResult.isValid() && !visionResult.getDetectorResults().isEmpty()) {
            LLResultTypes.DetectorResult bestDetector = null;
            double bestScore = 0.0;

            for (Object obj : visionResult.getDetectorResults()) {
                LLResultTypes.DetectorResult detector = (LLResultTypes.DetectorResult) obj;
                String detectedLabel = detector.getClassName();
                double confidence = detector.getConfidence();
                double area = detector.getTargetArea();

                if ("person".equalsIgnoreCase(detectedLabel)) {
                    double score = confidence * 0.6 + area * 0.4;
                    if (score > bestScore) {
                        bestScore = score;
                        bestDetector = detector;
                    }
                }
            }

            if (bestDetector != null) {
                double tx = bestDetector.getTargetXDegrees();
                double ty = bestDetector.getTargetYDegrees();
                double ta = bestDetector.getTargetArea();

                double nominalVoltage = 12.0;
                double currentVoltage = BigWheel.voltageSensor.getVoltage();
                double compensatedPower = Math.min(nominalVoltage / currentVoltage, 1.0);

                BigWheel.rotateFlyWheel1(compensatedPower);
                BigWheel.rotateFlyWheel2(-compensatedPower);

                if (tx > errorOffset + 0.01) {
                    BigWheel.shooterPanRight(tx * tXErrorMultiplier);
                    telemetry.addLine("Pan Right: " + tx);
                } else if (tx < -(errorOffset + 0.01)) {
                    BigWheel.shooterPanLeft(tx * tXErrorMultiplier);
                    telemetry.addLine("Pan Left: " + tx);
                } else {
                    BigWheel.setShooterPanStop();
                    telemetry.addLine("Pan Stop");
                }

                if (ty > errorOffset + 0.01) {
                    BigWheel.shooterTiltUp(ty * tYErrorMultiplier);
                    telemetry.addLine("Tilt Up: " + ty);
                } else if (ty < -(errorOffset + 0.01)) {
                    BigWheel.shooterTiltDown(ty * tYErrorMultiplier);
                    telemetry.addLine("Tilt Down: " + ty);
                } else {
                    BigWheel.setShooterTiltStop();
                    telemetry.addLine("Tilt Stop");
                }

                if (ta > 10 && ta < 40 && Math.abs(tx) < errorOffset && Math.abs(ty) < errorOffset) {
                    loadState = LoadStates.LOAD;
                    //BigWheel.unloadDisc();
                    telemetry.addLine("Load and Fire!");
                }

                telemetry.addData("Detected", bestDetector.getClassName());
                telemetry.addData("Confidence", bestDetector.getConfidence());
                telemetry.addData("Score", bestScore);
                telemetry.addData("TX", tx);
                telemetry.addData("TY", ty);
                telemetry.addData("TA", ta);
                telemetry.addData("Battery Voltage", currentVoltage);
                telemetry.addData("Flywheel Power", compensatedPower);
            } else {
                BigWheel.setShooterPanStop();
                BigWheel.setShooterTiltStop();
                BigWheel.stopFlyWheel1();
                BigWheel.stopFlyWheel2();
                telemetry.addLine("No confident person detected.");
            }

            telemetry.update();
        }
    }

}
