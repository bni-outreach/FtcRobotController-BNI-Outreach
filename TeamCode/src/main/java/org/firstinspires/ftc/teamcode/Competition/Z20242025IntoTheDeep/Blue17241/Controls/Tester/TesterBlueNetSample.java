package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Tester:Blue:Net:Sample", group = "Testers")
public class TesterBlueNetSample extends TesterBlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStartUp();

        resetHeading();
        currentHeading = getHeading();
        odo.update();
        waitForStart();

        //drive methods are pulled from Drivetrain


        //drive code
        while (opModeIsActive()) {

            strafeRightPinpoint(0.5, 0.1);
            sleep(1300);

            driveForwardPinpoint(0.5, 3.75);
            sleep(1300);

            driveBackPinpoint(.5, 8);
            sleep(1300);

            Bot.rotateRight(.5 ,7);
            sleep(1300);

            odo.update();
            requestOpModeStop();

        }
        idle();
    }
}

