package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Blue.Paths.Obs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Auto.Blue.BlueAlliance;

@Disabled
@Autonomous(name = "Blue:Observation:Park")
public class BlueObservationPark extends BlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException{

        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();

        ITDBot.retractIntake();

        while (opModeIsActive()){

            ITDBot.strafeLeft(1, .2);
            ITDBot.driveForward(1, 2.5);

            requestOpModeStop();

        }
        idle();
    }
}