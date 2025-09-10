package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Blue.Paths.Net;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Blue.BlueAlliance;

@Disabled
@Autonomous(name = "Blue:Net:TopBucket:LowBarTouch")
public class BlueNetTopBucketLowBar extends BlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();

        ITDBot.retractIntake();

        while (opModeIsActive()) {

// Go to net and place sample
            ITDBot.strafeLeft(0.5, 1.6);
            sleep(100);
            ITDBot.driveBack(0.5, .1);
            bucketDumpTopLevelOne();

// Move to face bar
            ITDBot.rotateRight(0.5,1);
// Go to bar and touch
            ITDBot.strafeLeft(0.5,3);
            sleep(50);
            ITDBot.driveForward(0.5,2);
           // ITDBot.climbingLiftUp(0.5);

            requestOpModeStop();
        }
        idle();
    }
}
