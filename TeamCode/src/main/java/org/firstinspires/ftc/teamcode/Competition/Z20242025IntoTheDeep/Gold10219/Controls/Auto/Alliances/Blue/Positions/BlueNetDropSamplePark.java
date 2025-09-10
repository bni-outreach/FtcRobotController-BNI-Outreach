package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue.Positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Alliances.Blue.AutoBlueAlliance;

@Disabled
@Autonomous(name = "Blue:Net:DropSample:Park", group = "blue")
public class BlueNetDropSamplePark extends AutoBlueAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStart();

        //wait for player to press start
        waitForStart();
        while (opModeIsActive()) {
            //TEST the PARK

            // START AUTO PATH SEQUENCE
            Bot.strafeRightInches(0.5, 4.8);
            sleep(500);
            safeExtendAndRaise();
            sleep(500);
            Bot.driveForwardInches(0.5, 54);
            sleep(500);
            Bot.rotateLeftDegrees(0.5, 36);
            sleep(500);
            Bot.strafeRightInches(0.5, 4.8);
            sleep(500);
            Bot.driveForwardInches(.5, 6);
            sleep(1000);
            dropAndRetreatFromBucket();
            sleep(500);
            Bot.rotateRightDegrees(0.5, 36);
            sleep(500);
            Bot.rotateLeftDegrees(0.5, 180);
            Bot.driveBackInches(0.5,108);


            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
        idle();
    }
}