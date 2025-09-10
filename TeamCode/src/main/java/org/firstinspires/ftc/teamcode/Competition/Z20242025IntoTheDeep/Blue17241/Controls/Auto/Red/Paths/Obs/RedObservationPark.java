package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Red.Paths.Obs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Red.RedAlliance;

@Disabled
@Autonomous(name = "Red:Observation:ObsPark")
public class RedObservationPark extends RedAlliance {
    @Override
    public void runOpMode() throws InterruptedException{

        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();

        ITDBot.retractIntake();

        while (opModeIsActive()){

            ITDBot.strafeRight(1, .2);
            ITDBot.driveBack(1, 2.2);

            requestOpModeStop();

        }
        idle();
    }
}