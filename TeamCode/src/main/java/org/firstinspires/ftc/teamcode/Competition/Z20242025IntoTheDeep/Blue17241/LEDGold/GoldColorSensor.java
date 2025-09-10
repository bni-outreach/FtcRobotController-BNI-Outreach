package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.LEDGold;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class GoldColorSensor {
    public HardwareMap hwBot = null;
    public CRServo intake = null;
    public Servo rotator = null;
    public RevColorSensorV3 sensor = null;

//    IndicatorStrip indicator = new IndicatorStrip();

    double intakeInPower = 1;
    double intakeOutPower = .5;
    double intakeRotatorCenter = 0.4044;
    double intakeRotatorStep = 0.005;
    double sampleSecuredDistance = 1.3;

//    double intakeMaxRight = 0.75;
//    double intakeMaxLeft = 0.1083;

    IntakeDirections state = IntakeDirections.STOP;

    double position = 0.5;

    //public Intake() {}

    public void initIntake(HardwareMap hwMap) {
        hwBot = hwMap;

        sensor = hwBot.get(RevColorSensorV3.class, "sample_sensor");

//        indicator.initIndicatorStrip(hwBot);
    }

    public void stateCheck() {
        if (state == IntakeDirections.IN) {
            intakeUntilSample();
        } else if (state == IntakeDirections.OUT) {
            dropSample();
        }
    }

    public enum IntakeDirections{
        IN,
        OUT,
        STOP;
    }
    public void start(IntakeDirections direction) {
        switch (direction) {
            case IN:
                intake.setPower(intakeInPower);
                break;
            case OUT:
                intake.setPower(-intakeOutPower);
                break;
            case STOP:
                intake.setPower(0);
                break;
        }
    }

    public void stop() {
        state = IntakeDirections.STOP;
        intake.setPower(0);
    }

    public void intakeUntilSample() {
        double distance = sensor.getDistance(DistanceUnit.INCH);
        if (distance > sampleSecuredDistance) {
            start(IntakeDirections.IN);
            state = IntakeDirections.IN;
        } else {
            stop();
            state = IntakeDirections.STOP;

            calcIntake();
        }
//
//        while (distance > sampleSecuredDistance && Op) {
//            start(IntakeDirections.IN);
//            isCollecting = true;
//            if (distance <= sampleSecuredDistance) break;
//        }
//        stop();
    }

    public void calcIntake() {
//        NormalizedRGBA colors = sensor.getNormalizedColors();
//        double red = colors.red;
//        double green = colors.green;
//        double blue = colors.blue;
//
//        RevBlinkinLedDriver.BlinkinPattern color;
//
//        double max = Math.max(red, Math.max(green, blue));
//
//        if (max == red) color = RevBlinkinLedDriver.BlinkinPattern.RED;
//        else if (max == blue) color = RevBlinkinLedDriver.BlinkinPattern.BLUE;
//        //TODO: Add calc for yellow
//        else color = RevBlinkinLedDriver.BlinkinPattern.BLACK;
//
//        indicator.capture(color);
    }

    public void dropSample() {
        double distance = sensor.getDistance(DistanceUnit.INCH);
        if (distance < sampleSecuredDistance) {
            start(IntakeDirections.OUT);
            state = IntakeDirections.OUT;
        } else {
            stop();
            state = IntakeDirections.STOP;
        }
    }
}
