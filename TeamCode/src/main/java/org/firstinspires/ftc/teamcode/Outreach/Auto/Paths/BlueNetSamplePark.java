package org.firstinspires.ftc.teamcode.Outreach.Auto.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Outreach.Auto.BlueAlliance;

//@Disabled
@Autonomous(name = "Test:Blue:Net:Sample:Park")
public class BlueNetSamplePark extends BlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStartUp();

        waitForStart();

        //drive methods are pulled from Drivetrain
        while (opModeIsActive()) {

            //drive code
            while (opModeIsActive()) {

                ProgramBot.strafeRight(1, .15);
                ProgramBot.driveForward(1, 3.85);
                sleep(1300);

                //Code for if partner can move away from wall
                ProgramBot.strafeRight(1, .15);
                ProgramBot.driveBack(1, 8);

                requestOpModeStop();

            }
            idle();
        }
    }
}

