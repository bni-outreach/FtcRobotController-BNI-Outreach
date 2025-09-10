//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@Disabled
@TeleOp(name = "B - Grabber Rotate Tester", group = "testers")
public class GrabberRotateTester extends OpMode {

    public CompBot Bot = new CompBot();

    Grabber grabber = new Grabber();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        grabber.initGrabber(hardwareMap);
    }

    @Override
    public void loop() {
        rotateControl();
        telemetry();
    }

    public void rotateControl() {
        if (gamepad1.right_bumper) {
            grabber.rotateRight();
        } else if (gamepad1.left_bumper) {
            grabber.rotateLeft();
        }
    }

    public void telemetry() {
        telemetry.addData("Rotate Angle: ", grabber.rotate .getPosition());
        telemetry.update();
    }

}
