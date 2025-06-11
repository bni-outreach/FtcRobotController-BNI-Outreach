package org.firstinspires.ftc.teamcode.Outreach.Robots;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Outreach.Drivetrains.RingRangerMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;


public class RingRangerBot extends RingRangerMecanumDrive {

    //hardware constructors
    public HardwareMap hwBot  =  null;


    public VoltageSensor voltageSensor = null;

    public double launchCoefficient;

//GYRO INITIALIZATION

    // Color and Distance Hardware & Variables
    public float hsvValues[] = {0F, 0F, 0F};
    public final double SCALE_FACTOR = 1;
//    Under 100 = red tape
    public static final int WOBBLE_ARM_RAISE_THRESHOLD = 170;
//    Looks for > 200 with blue tape
    public static final int WOBBLE_ARM_LOWER_THRESHOLD = 100;
    //  Camera Initialization

    public OpenCvCamera webcam;

//    Servo WobbleArm = null;
    Servo WobbleGrab = null;
    Servo Camera = null;
    public Servo ServoRingPusher = null;
    public Servo RingMag = null;
    public CRServo IntakeCorrector = null;

    public double WobbleGrabOpenPos = .59;
    public double WobbleGrabClosePos = 0.116;
    //Blue Left:
    //was at .2
    public double CameraServoPosBlueLeft = 0.2;
    //Blue Right:
    public double CameraServoPosBlueRight = 0.602;
    //Launcher Motor:
    public DcMotor IntakeMotor = null;
    public double RingPushPos = 0.26;
    //.196 before (Mar 3, 2021 3:40pm)
    public double RingPullPos = 0.40;
    //.460 before (Mar 3, 2021 3:40pm)
    public double RingMagUpPos = 0.2;
    //.13 before (Mar 3, 2021 3:42pm)
    public double RingMagDownPos = 0.051;
    //.058 before (Mar 3, 2021 3:42pm)
    public double RingMagUpAuto = 0.217;

    public double DeltaRing = Math.abs(RingPullPos - RingPushPos);
    int numLoops = 7;
    public double ringIncrement = DeltaRing/numLoops;

    public double DeltaRingMag = Math.abs(RingMagUpPos - RingMagDownPos);
    int numLoopsMag = 9;
    public double ringIncrementMag = DeltaRingMag/numLoopsMag;

//    Wobble Arm Motor Data
    public DcMotor WobbleArmMotor = null;
    public DcMotorEx launcherMotor1 = null;
    public DcMotorEx launcherMotor2 = null;
    public double velocity = 1600;
    public boolean wobbleArmRaiseEngage;
    public boolean wobbleArmLowerengage;

    public int rapidFireRing = 0;
    public int rapidPushTimer = 150; //200 before
    public int rapidPullTimer = 175; //150 before
    public boolean launchModePush = false;
    public boolean launchModePull = false;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    public ElapsedTime wobbleArmTimer;
    public ElapsedTime ringPusherTimer;

    //LabBot constructor
    public RingRangerBot() {}

    public void initRobot(HardwareMap hardwareMap, String startPosition, String mode){

        hwBot = hardwareMap;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        launchCoefficient = 12 / voltageSensor.getVoltage();
        WobbleGrab = hwBot.get(Servo.class, "wobble_grab");
        WobbleGrab.setDirection(Servo.Direction.FORWARD);
        RingMag = hwBot.get(Servo.class, "ring_mag");
        ServoRingPusher = hwBot.get(Servo.class, "servo_ring_pusher");
        Camera = hwBot.get(Servo.class, "camera_servo");
        Camera.setDirection(Servo.Direction.FORWARD);

        switch (startPosition) {
            case "BlueLeft":
                Camera.setPosition(CameraServoPosBlueLeft);
                break;
            case "BlueRight":
                break;
            case "RedLeft":
                break;
            case "RedRight":
                break;

        }

        blinkinLedDriver = hwBot.get(RevBlinkinLedDriver.class, "Blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
        blinkinLedDriver.setPattern(pattern);

        IntakeCorrector = hardwareMap.crservo.get ("intake_corrector");
        IntakeCorrector.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeCorrector.setPower(0);



        // define motors for robot
        frontLeftMotor=hwBot.dcMotor.get("front_left_motor");
        frontRightMotor=hwBot.dcMotor.get("front_right_motor");
        rearLeftMotor = hwBot.dcMotor.get("rear_left_motor");
        rearRightMotor = hwBot.dcMotor.get("rear_right_motor");


        IntakeMotor = hwBot.dcMotor.get("intake_motor");

        WobbleArmMotor = hwBot.dcMotor.get("wobble_arm_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        launcherMotor1 = hwBot.get(DcMotorEx.class, "launcher_motor_1");
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor2 = hwBot.get(DcMotorEx.class, "launcher_motor_2");
        launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        WobbleArmMotor.setDirection(DcMotor.Direction.FORWARD);
        WobbleArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArmRaiseEngage = false;
        wobbleArmLowerengage = false;


        //Initialize Motor Run Mode for Robot
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//      Timers
        wobbleArmTimer = new ElapsedTime();
        wobbleArmTimer.reset();

        ringPusherTimer = new ElapsedTime();
        ringPusherTimer.reset();

    }

    public void WobbleOpen(){
        WobbleGrab.setPosition(WobbleGrabOpenPos);
    }
    public void WobbleClosed(){
        WobbleGrab.setPosition(WobbleGrabClosePos);
    }

    public void LauncherOn(double power) {
        launcherMotor1.setVelocity(power);
        launcherMotor2.setVelocity(power);
     }
    public void LauncherOff(double power) {
        launcherMotor1.setPower(power);
        launcherMotor2.setPower(power);
    }

    public void IntakeOn(double power){
        IntakeMotor.setPower(power);
    }
    public void IntakeOff(double power){
        IntakeMotor.setPower(power);
    }
    public void WobbleArmRaised(double power){WobbleArmMotor.setPower(power);}
    public void WobbleArmLower(double power){WobbleArmMotor.setPower(-power);}

    public void SpinInIntakeCorrector(){
        IntakeCorrector.setPower(1);
    }

    public void StopIntakeCorrector(){
        IntakeCorrector.setPower(0);
    }

    public void FirstLaunch(){
        RingPull();
        ringPusherTimer.reset();
        launchModePush = true;
        launchModePull = false;
        rapidFireRing++;
    }
    public void rapidFire(){
        if (rapidFireRing == 0){
            ringPusherTimer.reset();
            rapidFireRing++;
            launchModePush = true;
            launchModePull = false;
        }
        else if (rapidFireRing == 1){
            FirstLaunch();
        }
        else if (rapidFireRing == 2){
            rapidFireOp();
        }
        else if (rapidFireRing == 3){
            rapidFireOp();
        }
        else if (rapidFireRing == 4){
            rapidFireOp();

        }
        else if (rapidFireRing == 5){
            rapidFireOp();
        }
        else if (rapidFireRing == 6){
            rapidFireOp();
        }
        else{
            RingPull();
        }

    }

    public void rapidFireOp(){
        if (ringPusherTimer.milliseconds() > rapidPushTimer && launchModePush == true){
            RingPush();
            ringPusherTimer.reset();
            launchModePush = false;
            launchModePull = true;
        }
        else if (ringPusherTimer.milliseconds() > rapidPullTimer && launchModePull == true){
            RingPull();
            ringPusherTimer.reset();
            rapidFireRing++;
            launchModePush = true;
            launchModePull = false;
        }
    }

    public void WobbleArmStopMotors () {
        WobbleArmMotor.setPower(0);
    }
    //        Pulls back pusher

    public void RingPush() {
        ServoRingPusher.setPosition(RingPushPos);
    }
    //        Pushes ring to launch!  Zoom Zoom
    public void RingPull() {
        ServoRingPusher.setPosition(RingPullPos);
    }

    public void RingPullIncrement(){
        if (ServoRingPusher.getPosition() <= RingPullPos) {
            ServoRingPusher.setPosition(ServoRingPusher.getPosition() + ringIncrement);
        }
    }
    public void RingMagIncrement(){
        if (RingMag.getPosition() <= RingMagUpPos){
            RingMag.setPosition(RingMag.getPosition() + ringIncrementMag);
        }
    }
    public void RingMagUp(){
        RingMag.setPosition(RingMagUpAuto);
    }
    public void RingMagDown(){
        RingMag.setPosition(RingMagDownPos);
    }


}
