package org.firstinspires.ftc.teamcode.Outreach.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Outreach.Robots.MecanumBot;

public abstract class AutoMain extends LinearOpMode {

    // Constructor for the Competition Robot for the Blue Team
    public MecanumBot ProgramBot = new MecanumBot();

    // Helper Method for Initializing, Setting LinearOp, and Updating Telemetry
    public void autoStartUp(){
        ProgramBot.initRobot(hardwareMap);
        ProgramBot.setLinearOp(this);
        telemetry.addLine("Awaiting Start");
        telemetry.update();
    }



}
