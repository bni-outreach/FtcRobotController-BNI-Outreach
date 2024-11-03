package org.firstinspires.ftc.teamcode.Outreach.Auto.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Outreach.Auto.BlueAlliance;

@Disabled
@Autonomous(name = "Test:Blue:Net:ParkOnly")
public class BlueNetPark extends BlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException{

        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();

        while(opModeIsActive()){

            sleep(25000);

            Bot.driveForward(1, 2);

            requestOpModeStop();

        }
        idle();
    }
}
