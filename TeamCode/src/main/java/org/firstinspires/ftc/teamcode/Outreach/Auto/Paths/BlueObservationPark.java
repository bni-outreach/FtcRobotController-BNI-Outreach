package org.firstinspires.ftc.teamcode.Outreach.Auto.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Outreach.Auto.BlueAlliance;

//@Disabled
@Autonomous(name = "Test:Blue:Observation:Auto")
public class BlueObservationPark extends BlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException{

        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();


        while (opModeIsActive()){

            ProgramBot.strafeLeft(1, .2);
            ProgramBot.driveForward(1, 2.5);

            requestOpModeStop();

        }
        idle();
    }
}