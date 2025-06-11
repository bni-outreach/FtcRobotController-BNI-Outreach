package org.firstinspires.ftc.teamcode.Outreach.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGBLight {

    public HardwareMap hwBot = null;
    public Servo rgbLight = null;

    // Enum For Color Name with associated servo positions
    public enum ColorOptions {
        RED(0.279),
        ORANGE(.333),
        YELLOW(.388),
        GREEN(.5),
        BLUE(.611),
        PURPLE(.722),
        OFF(0);

        public final double VALUE;


        ColorOptions(double value) {
            this.VALUE = value;
        }

        public double getValue() {
            return this.VALUE;
        }
    }

    public void initRGBLight(HardwareMap hwMap) {
        hwBot = hwMap;
        rgbLight = hwBot.servo.get("indicator");
        rgbLight.setDirection(Servo.Direction.FORWARD);
    }

    public void setColor(ColorOptions option) {
        rgbLight.setPosition(option.getValue());
    }
    
    
}
