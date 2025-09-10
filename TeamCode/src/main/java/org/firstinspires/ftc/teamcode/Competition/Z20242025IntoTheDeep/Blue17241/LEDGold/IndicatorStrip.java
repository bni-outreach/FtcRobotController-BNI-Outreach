package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.LEDGold;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Robots.ITDBot;

@Disabled
public class IndicatorStrip {
    public ITDBot ITDBot;
    public HardwareMap hwBot;
    public RevBlinkinLedDriver strip;

    //Timer
    private ElapsedTime time = new ElapsedTime();

    public IndicatorStrip() {}

    public void initIndicatorStrip(HardwareMap hwMap) {
        hwBot = hwMap;

        strip = hwBot.get(RevBlinkinLedDriver.class, "indicator_strip");
    }

    public void capture(RevBlinkinLedDriver.BlinkinPattern color) {
        int state = 0;
        int iters = 0;

        switch (state) {
            case 0:
                if (iters == 0) {
                    strip.setPattern(color);
                    time.reset();
                    state++;
                    break;
                } else if (time.time() >= 1.0) {
                    strip.setPattern(color);
                    time.reset();
                    state++;
                    break;
                }
            case 1:
                if (time.time() >= 1.0) {
                    strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    time.reset();
                    iters ++;
                    if (iters >= 3) {
                        state++;
                    } else {
                        state--;
                    }
                }
                break;
        }
    }

    public void off() {
        strip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
