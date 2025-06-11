package org.firstinspires.ftc.teamcode.Outreach.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Outreach.Robots.VisionBot;

@Disabled
@TeleOp(name = "Limelight Tester")
public class LimeTester extends LinearOpMode {

    // Instantiate Objects for Vision Tracking and Robot Driving
    public Vision visionCam = new Vision();
    public VisionBot Bot = new VisionBot();


    @Override
    public void runOpMode() throws InterruptedException
    {
        autoStart();

        telemetry.setMsTransmissionInterval(11);

        visionCam.startCam();

        waitForStart();

        Pipeline[] samplesToTest = new Pipeline[]{Pipeline.RED, Pipeline.BLUE, Pipeline.YELLOW};
        int closestPipeline = visionCam.getClosestPipeline(samplesToTest);

        visionCam.setPipeline(closestPipeline);

        while (opModeIsActive()) {

            telemetry.addLine("OpMode Active");

            visionCam.getResult();

            if (visionCam.lastResultValid()) {
                telemetry.addLine("Result Valid");
                double[] offsets = visionCam.getOffsets();
                Pose3D pose = visionCam.getPose();

                double tx = offsets[0];
                double ty = offsets[1];

                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);

                double rotationSpeed = Math.abs(tx) * visionCam.errorMultiplier;

                if (rotationSpeed > visionCam.minimumCommand && rotationSpeed < visionCam.maximumCommand) {
                    rotationSpeed = Range.clip(rotationSpeed, visionCam.minimumCommand, visionCam.maximumCommand);
                }
                else if (rotationSpeed > visionCam.maximumCommand) {
                    rotationSpeed = visionCam.maximumCommand;
                }
                else {
                    rotationSpeed = visionCam.minimumCommand;
                }

                if (tx < 0 - visionCam.errorOffset) {
                    Bot.rotateLeft(rotationSpeed);
                }
                else if (tx > 0 + visionCam.errorOffset) {
                    Bot.rotateRight(rotationSpeed);
                }
                else {
                    Bot.stopMotors();
                }

            }
            else {
                telemetry.addLine("Invalid Result");
            }

            telemetry.update();
        }

        visionCam.stop();
    }

    // Helper Methods

    public void autoStart() {
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);

        visionCam.initVision(hardwareMap, true, 4, "tester_");
        visionCam.setLinearOp(this);
    }

}
