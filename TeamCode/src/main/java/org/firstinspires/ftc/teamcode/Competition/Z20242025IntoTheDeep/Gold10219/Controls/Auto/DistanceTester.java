package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "DistanceTester")
public class DistanceTester extends AutoMain{

    @Override
    public void runOpMode() throws InterruptedException {

        //Global Method for Initializing Auto
        autoStart();

        //wait for player to press start
        waitForStart();
        while (opModeIsActive()) {

            //START AUTO PATH SEQUENCE
//            //1 Rotation is 6"
//            Bot.strafeRight(0.5, 1);
//            sleep(500);
//            //.5 Rotation is 3"
//            Bot.strafeLeft(0.5, 0.5);
//            sleep(1000);
//
//            //1 Rotation is 6"
//            Bot.driveForward(0.5, 1);
//            sleep(500);
//            //.5 Rotation is 3"
//            Bot.driveBack(0.5, 0.5);
//            sleep(500);

            //1 Rotation is 45 deg
            Bot.rotateLeftDegrees(0.5, 45);
            sleep(500);
            //.5 Rotation is 22.5 deg
            Bot.rotateRightDegrees(0.5, 22.5);
            sleep(500);

            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
        idle();
    }
}
