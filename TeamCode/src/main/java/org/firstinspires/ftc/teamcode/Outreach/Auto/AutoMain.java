package org.firstinspires.ftc.teamcode.Outreach.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Outreach.Robots.BigBerthaBot;

public abstract class AutoMain extends LinearOpMode {

    // Constructor for the Competition Robot for the Blue Team
    public BigBerthaBot ProgramBot = new BigBerthaBot();

    // Helper Method for Initializing, Setting LinearOp, and Updating Telemetry
    public void autoStartUp(){
        ProgramBot.initRobot(hardwareMap);
        ProgramBot.setLinearOp(this);
        telemetry.addLine("Awaiting Start");
        telemetry.update();
    }



}
