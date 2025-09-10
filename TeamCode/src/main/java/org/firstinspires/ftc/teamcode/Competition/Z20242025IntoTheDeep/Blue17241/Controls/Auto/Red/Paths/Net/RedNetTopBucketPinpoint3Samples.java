package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Red.Paths.Net;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Red.RedAlliance;

@Disabled
@Autonomous(name ="Red:Net:TopBucket:Pinpoint:3Samples")
public class RedNetTopBucketPinpoint3Samples extends RedAlliance {
    //@Override
    public void runOpMode() throws InterruptedException{
        autoStartUp();
        resetHeadingPinpoint();
        currentHeading = getHeadingPinpoint();
        odo.update();
        waitForStart();

        while(opModeIsActive()) {

            redNetPreloadSpikeOne();

            //move to second field sample
            redNetSpikeTwo();

            requestOpModeStop();
        }
    }
}
