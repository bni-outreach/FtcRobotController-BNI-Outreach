package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Alliances.Red.Positions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Alliances.Red.AutoRedAlliance;

@Disabled
@Autonomous(name = "Red:Observation:Park", group = "red")
public class RedObservationPark extends AutoRedAlliance {

    @Override
    public void runOpMode() throws InterruptedException {

        //Global Method for Initializing Auto
        autoStart();

        //wait for player to press start
        waitForStart();
        while(opModeIsActive()) {

            // START AUTO PATH SEQUENCE
            Bot.strafeRightInches(0.5, 4.8);
            sleep(500);
            Bot.driveBackInches(0.5, 48);

            // END AUTO PATH SEQUENCE
            requestOpModeStop();
        }
        idle();
    }
}
