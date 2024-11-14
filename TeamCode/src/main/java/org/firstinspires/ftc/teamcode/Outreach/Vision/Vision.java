package org.firstinspires.ftc.teamcode.Outreach.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.ArrayList;

public class Vision {

    public HardwareMap hwBot = null;
    public Limelight3A cam = null;
    public LinearOpMode LinearOp = null;

    public double errorMultiplier = 0.025;
    public double errorOffset = 4;
    public double minimumCommand = 0.1;
    public double maximumCommand = 0.4;

    public boolean captureSnapshots = false;
    public int snapshotLimit = 0;
    public String snapshotPrefix = "pov_";
    private int numSnapshots = 0;

    private LLResult result = null;

    public Vision() {}

    public void setLinearOp(LinearOpMode LinearOp) {
        this.LinearOp = LinearOp;
    }

    public void initVision(HardwareMap hwMap, boolean captureSnapshots, int snapshotLimit, String snapshotPrefix) {
        hwBot = hwMap;
        this.captureSnapshots = captureSnapshots;
        this.snapshotLimit = snapshotLimit;
        this.snapshotPrefix = snapshotPrefix;

        cam = hwBot.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
    }

    // Helper method to start the Limelight camera
    public void startCam() {
        cam.start();
        LinearOp.telemetry.addLine("Vision Started");
        LinearOp.telemetry.update();
    }

    // Setter Method to define the correct Limelight pipeline
    public void setPipeline(int pipeline) {

        cam.pipelineSwitch(pipeline);
    }

    // Getter Method to get the latest results from the pipeline
    public void getResult() {

        result = cam.getLatestResult();
    }

    // Helper Method for Boolean test on valid results
    public boolean lastResultValid() {

        return result != null && result.isValid();
    }

    public Pose3D getPose() {
        if (result != null) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    public double[] getOffsets() {

        if (result != null && result.isValid()) {
            if (captureSnapshots && numSnapshots < snapshotLimit) {
                captureSnapshot();
                numSnapshots ++;
            }

            double[] coords = new double[3];
            coords[0] = result.getTx();
            coords[1] = result.getTy();
            coords[2] = result.getTa();
            return coords;
        }
        return null;
    }

    public int getClosestPipeline(Pipeline[] pipelines) {

        //Define arraylist of sample pipeline results
        ArrayList<PipelineResults> results = new ArrayList<PipelineResults>();

        //For each desired pipeline, determine tx, ty, and ta values, then store in arrayList
        for (Pipeline pipeline : pipelines) {
            int pipelineId = 0;
            switch (pipeline) {
                case RED:
                    pipelineId = 0;
                    break;
                case BLUE:
                    pipelineId = 1;
                    break;
                case YELLOW:
                    pipelineId = 2;
                    break;

            }
            //Set pipeline
            setPipeline(pipelineId);

            //Get pipeline result
            getResult();

            if (result != null && result.isValid()) {

                //Store double values
                double tx = getOffsets()[0];
                double ty = getOffsets()[1];
                double ta = getOffsets()[2];

                //Instantiate new result object with tx, ty, ta, and pipeline id
                PipelineResults result = new PipelineResults(tx, ty, ta, pipelineId);

                //Add result to array
                results.add(result);
            }
        }

//        LinearOp.telemetry.update();

        //For each result, determine smallest ta value, then output pipeline id
        double closestTa = 0;
        int closestPipeline = 0;
        for (PipelineResults result : results) {
            if (result.ta > closestTa) {
                closestTa = result.ta;
                closestPipeline = result.pipeline;
            }
        }

        return closestPipeline;
    }

    public void captureSnapshot() {
        LLResult res = cam.getLatestResult();
        if (res != null && res.isValid()) {
            cam.captureSnapshot("pov_" + snapshotPrefix);
        }
    }

    public void stop() {
        cam.stop();
    }


}
