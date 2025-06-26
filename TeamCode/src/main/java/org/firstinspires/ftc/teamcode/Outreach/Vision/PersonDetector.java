package org.firstinspires.ftc.teamcode.Outreach.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Outreach.Controls.BigWheelTeleOp;
import org.firstinspires.ftc.teamcode.Outreach.Robots.BigWheelBot;

@Disabled
@TeleOp(name = "Person Detector Tester")
public class PersonDetector extends OpMode {

    //Limelight Variables
    public double tXErrorMultiplier = .02;
    public double tYErrorMultiplier = .015;
    public double errorOffset = 7;
    public double minimumCommand = 0.1;
    public double maximumCommand = 0.4;

    //Limelight Results
    public LLResult result = null;
    public LLResultTypes.DetectorResult detector = null;

    public BigWheelBot BigWheel = new BigWheelBot();

    @Override
    public void init() {
        BigWheel.initLimelight(hardwareMap);
        BigWheel.initVoltageSensor(hardwareMap);
    }

    @Override
    public void start() {
        BigWheel.cam.start();
        BigWheel.cam.pipelineSwitch(4);
    }


    @Override
    public void loop() {
        autoTurret();
        //autoTurretWithVisionModel();

    }


    public void getCamResult() {
        result = BigWheel.cam.getLatestResult();

    }

    public boolean lastResultValid() {
        return result != null && result.isValid();
    }

    public void autoTurret() {

        getCamResult();

        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            detector = (LLResultTypes.DetectorResult) result.getDetectorResults().get(0);

            telemetry.addData("Class Name: ", detector.getClassName());
            telemetry.addData("Detector Results:", detector.getConfidence());
            telemetry.addData("Target Area:", detector.getTargetArea());
            telemetry.addData("Target Corners:", detector.getTargetCorners());
            telemetry.addData("Target X Degrees:", detector.getTargetXDegrees());
            telemetry.addData("Target Y Degrees:", detector.getTargetYDegrees());
        }
        else {
            telemetry.addLine("No valid detector result.");
        }

        telemetry.addData("Tx:", result.getTx());
        telemetry.addData("Ty:", result.getTy());
        telemetry.addData("Ta:", result.getTa());
        telemetry.update();


    }

}