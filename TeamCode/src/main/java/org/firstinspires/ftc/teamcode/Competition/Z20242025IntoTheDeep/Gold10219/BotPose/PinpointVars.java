package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BotPose;

public class PinpointVars {

    public Offsets Offsets = new Offsets();
    public Directions Directions = new Directions();

    public PinpointDriver.GoBildaOdometryPods resolution = PinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public PinpointVars() {}

    public static final class Offsets {
        public double x = -66.88;
        public double y = -168.28;
    }

    public static final class Directions {
        public PinpointDriver.EncoderDirection x = PinpointDriver.EncoderDirection.REVERSED;
        public PinpointDriver.EncoderDirection y = PinpointDriver.EncoderDirection.FORWARD;
    }
}
