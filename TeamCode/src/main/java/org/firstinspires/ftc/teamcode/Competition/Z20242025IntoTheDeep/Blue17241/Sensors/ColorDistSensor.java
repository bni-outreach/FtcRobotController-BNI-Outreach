package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorDistSensor {

    // Instance Variables
    public HardwareMap hwBot = null;
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    // Class Constructor
    public ColorDistSensor() {}

    //Instance Methods
    public float hsvValues[] = {0F, 0F, 0F};
    public final float values[] = hsvValues;
    public final double SCALE_FACTOR = 255;

    public void initColorDistSensor(HardwareMap hwMap) {
        hwBot = hwMap;
        //distanceSensor = hwBot.get(DistanceSensor.class, "sample_sensor_dist");
        colorSensor = hwBot.get(ColorSensor.class, "sample_sensor");
    }

    public void convertColors() {
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);
    }


}
