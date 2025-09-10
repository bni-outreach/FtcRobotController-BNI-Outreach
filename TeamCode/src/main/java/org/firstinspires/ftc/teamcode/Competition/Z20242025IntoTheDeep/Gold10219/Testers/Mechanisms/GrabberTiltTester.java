//Shortest file in this repo lol
package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Testers.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

@Disabled
@TeleOp(name = "B - Grabber Tilt Tester", group = "testers")
public class GrabberTiltTester extends OpMode {

    public CompBot Bot = new CompBot();

    Grabber grabber = new Grabber();



    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        grabber.initGrabber(hardwareMap);
        grabber.doTuck();
    }

    @Override
    public void loop() {
        tiltControl();
        telemetry();
        grabber.tiltStateCheck();
    }

    public void tiltControl() {
        if (gamepad1.dpad_up) {
            grabber.tiltUp();
        } else if (gamepad1.dpad_down) {
            grabber.tiltDown();
        } else if (gamepad1.a) {
            grabber.setGrabberState(Grabber.grabberStates.OUT);
        } else if (gamepad1.b) {
            grabber.setGrabberState(Grabber.grabberStates.DOWN);
        } else if (gamepad1.y) {
            grabber.setGrabberState(Grabber.grabberStates.MANUAL);
        }
    }

    public void telemetry() {
        telemetry.addData("Current IMU Angle: ", grabber.getTilt());
        telemetry.addData("Servo Position: ", grabber.tilt.getPosition());
        telemetry.addLine();
        telemetry.addData("Grabber State: ", grabber.grabberState);
        telemetry.addData("Tilt State: ", grabber.getTiltState());
        telemetry.addLine();
        telemetry.addData("IMU Angle: ", grabber.ang);
        telemetry.addData("Diff: ", grabber.diff);
        telemetry.addData("Normalized Diff: ", grabber.diff1);
        telemetry.addData("Pos Ch: ", grabber.pch);
        telemetry.addData("Current Servo Position: ", grabber.csp);
        telemetry.addData("New Servo Position: ", grabber.nsp);
        telemetry.addData("Clamped Servo Position: ", grabber.nsp2);
        telemetry.update();
    }

}
