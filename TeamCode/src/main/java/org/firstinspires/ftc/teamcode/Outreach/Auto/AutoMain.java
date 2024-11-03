package org.firstinspires.ftc.teamcode.Outreach.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Outreach.Robots.MecanumBot;

public abstract class AutoMain extends LinearOpMode {

    // Constructor for the Competition Robot for the Blue Team
    public MecanumBot Bot = new MecanumBot();

    // Helper Method for Initializing, Setting LinearOp, and Updating Telemetry
    public void autoStartUp(){
        Bot.initRobot(hardwareMap);
        Bot.setLinearOp(this);
        telemetry.addLine("Awaiting Start");
        telemetry.update();
    }



}
