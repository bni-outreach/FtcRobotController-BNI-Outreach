package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Blue.Paths.Net;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Blue.BlueAlliance;

@Disabled
@Autonomous(name = "Blue:Net:TopBucket:ObsPark")
public class BlueNetTopBucketObsPark extends BlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException{
        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();

        ITDBot.retractIntake();

        while(opModeIsActive()){

            sleep(25000);

            ITDBot.strafeRight(1, 1.6);
            ITDBot.driveBack(1, .1);
            bucketDumpTopLevelOne();

            //NOT Tested
            ITDBot.driveBack(1,1.5);
            ITDBot.rotateLeft(1, 1.75);
            ITDBot.driveForward(1, 5);

            requestOpModeStop();

    }
    idle();
}
}
