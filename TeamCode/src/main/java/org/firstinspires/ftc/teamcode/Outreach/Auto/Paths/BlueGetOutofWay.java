package org.firstinspires.ftc.teamcode.Outreach.Auto.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Outreach.Auto.BlueAlliance;

@Disabled
@Autonomous(name = "Test:Blue:GetOutofWay")
public class BlueGetOutofWay extends BlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException{

        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();


        //drive methods are pulled from Drivetrain
        while(opModeIsActive()){

            ProgramBot.strafeRight(1, 3);
            sleep(25000);
            requestOpModeStop();

        }
        idle();
    }
}
