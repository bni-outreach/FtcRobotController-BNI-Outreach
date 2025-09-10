package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Tester:Blue:FieldCentricPID", group = "Testers")
public class TesterAutoFieldCentricPID extends TesterBlueAlliance {
    @Override
    public void runOpMode() throws InterruptedException {

        // Global Method for Initializing Auto
        autoStartUp();

        resetHeading();
        currentHeading = getHeading();
        odo.update();

        waitForStart();


        while (opModeIsActive()) {

            // Insert Tester Code

            driveToPositionPID(0,15,0,.40);
            sleep(4000);



            requestOpModeStop();

        }
        idle();
    }



}

