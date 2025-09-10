package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight;

import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.ascentRExtendArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.ascentRExtendArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.ascentRGrabberOut;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.ascentRLowerArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.ascentRTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.chambers1RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.chambers1RaiseArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.holdAscentR;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.holdNets1;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1DropSample;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1DropSampleTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1ExtendArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1ExtendArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1Heading;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1ResetMechanisms;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1ResetMechanismsTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.nets1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.opModeStop;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.toAscentR;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Nets.AscentRight.PathStates.toNets1;

//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
//import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.O;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.Offsets;
//import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.Paths.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.Paths.HeadingTypes;

import java.util.HashMap;
import java.util.Map;

//@Autonomous(name = "Red Drop Sample Touch Ascent Right", group = "Auto - Red - Sample")
//public class RedDropSampleTouchAscentRight extends OpMode {
//    private final CompBot Bot = new CompBot();
//    private final CompBotVars vars = new CompBotVars();
//
//    private final Vision vision = new Vision();
//    private final Pinpoint pinpoint = new Pinpoint();
//    private final PoseHelper pose = new PoseHelper();
//
//    private final FieldPoses poses = new FieldPoses();
//
//    Grabber grabber = new Grabber();
//
//    PrimaryArm arm = new PrimaryArm();
//
//    private Pose startPose;
//
//    private Follower follower;
//
//    private Timer pathTimer;
//
//    private final Map<PathStates, Path> paths = new HashMap<>();
//    private PathStates pathState;
//
//    @Override
//    public void init() {
//        Bot.initRobot(hardwareMap);
//
//        pinpoint.setOp(this);
//        pinpoint.initPinpoint(hardwareMap);
//
//        vision.setOp(this);
//        vision.initVision(hardwareMap, pinpoint);
//
//        pose.setOp(this);
//        pose.setDevices(vision, pinpoint);
//
//        pathTimer = new Timer();
//
//        vision.start();
//
//        pose.updateLLUsage(false);
//
//        Pose2D currentPose = pose.getSmartPose(PoseHelper.Alliances.RED);
//        telemetry.addData("Pose X: ", currentPose.getX(DistanceUnit.INCH));
//        telemetry.addData("Pose Y: ", currentPose.getY(DistanceUnit.INCH));
//        telemetry.addData("Pose H: ", currentPose.getHeading(AngleUnit.DEGREES));
//
//        startPose = new Pose(
//                currentPose.getX(DistanceUnit.INCH),
//                currentPose.getY(DistanceUnit.INCH),
//                currentPose.getHeading(AngleUnit.RADIANS)
//        );
//
//        telemetry.addData("Start Pose: ", startPose);
//        telemetry.update();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
//        grabber.initGrabber(hardwareMap);
//
//        grabber.grab();
//        grabber.headStraight();
//        grabber.setGrabberState(Grabber.grabberStates.TUCK);
//        arm.setRetract();
//    }
//
//    public void start() {
//        grabber.grab();
//        grabber.headStraight();
//        grabber.setGrabberState(Grabber.grabberStates.TUCK);
//        arm.setRetract();
//        buildPaths();
//        setPathState(chambers1RaiseArm);
//    }
//
//    public void loop() {
//        pose.updatePose();
//        follower.update();
//        grabber.tiltStateCheck();
//        arm.rotationChecker();
//        autonomousPathUpdate();
//        tel();
//    }
//
//    Pose l = null;
//
//    public void tel() {
//        Pose2D current = pose.getPose();
//        telemetry.addData("PX: ", current.getX(DistanceUnit.INCH));
//        telemetry.addData("PY: ", current.getY(DistanceUnit.INCH));
//        telemetry.addData("PO: ", current.getHeading(AngleUnit.DEGREES));
//        telemetry.addLine();
//        telemetry.addData("pathState: ", pathState);
//        if (l != null) {
//            telemetry.addLine();
//            telemetry.addData("LX: ", l.getX());
//            telemetry.addData("LY: ", l.getY());
//            telemetry.addData("LH: ", l.getHeading());
//        }
//        telemetry.update();
//    }
//
//    private Path getPath(PathStates state) {
//        return O.req(paths.get(state));
//    }
//
//    public void buildPaths() {
//        paths.put(toNets1,
//                new EasySafePath(startPose, poses.Nets.Red,
//                        new Offsets().addY(vars.Chassis.FRONT_LENGTH).addX(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.Mechanisms.Grabber.AtChambers.OUT))
//                        .setHeading(HeadingTypes.CONSTANT, startPose));
//
//        paths.put(toAscentR,
//                new EasySafePath(getPath(toNets1).getLastControlPoint(), poses.Ascents.Pre.Red, poses.Ascents.Post.Red, poses.Ascents.Red.Right,
//                        new Offsets().remX(vars.Chassis.FRONT_LENGTH))
//                        .setHeading(HeadingTypes.LINEAR, getPath(toNets1), poses.Ascents.Red.Right, .35));
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case chambers1RaiseArm:
//                arm.setRotation(PrimaryArm.rotationStates.UP, 6, true);
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                setPathState(chambers1RaiseArmTimeout);
//                break;
//            case chambers1RaiseArmTimeout:
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(toNets1);
//                }
//                break;
//            case toNets1:
//                follower.followPath(getPath(toNets1));
//                setPathState(nets1Heading);
//                break;
//            case nets1Heading:
//                if (follower.getCurrentTValue() > 0.1) {
//                    ((EasySafePath) getPath(toNets1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Nets.Red);
//                    setPathState(holdNets1);
//                }
//                break;
//            case holdNets1:
//                if (!follower.isBusy()) {
//                    follower.holdPoint(
//                            new EasyPoint(poses.Nets.Red,
//                                    new Offsets().addY(vars.Chassis.FRONT_LENGTH).addX(vars.Chassis.FRONT_LENGTH).addY(vars.Mechanisms.Grabber.AtChambers.OUT).addX(vars.Mechanisms.Grabber.AtChambers.OUT)),
//                            poses.Nets.Red.getHeading()
//                    );
//                    setPathState(nets1Timeout);
//                }
//                break;
//            case nets1Timeout:
//                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
//                    setPathState(nets1ExtendArm);
//                }
//                break;
//            case nets1ExtendArm:
//                arm.setExtend();
//                setPathState(nets1ExtendArmTimeout);
//                break;
//            case nets1ExtendArmTimeout:
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(nets1LowerArm);
//                }
//                break;
//            case nets1LowerArm:
//                arm.setRotation(PrimaryArm.rotationStates.DOWN, 1, false);
//                setPathState(nets1LowerArmTimeout);
//                break;
//            case nets1LowerArmTimeout:
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(nets1DropSample);
//                }
//                break;
//            case nets1DropSample:
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                grabber.release();
//                setPathState(nets1DropSampleTimeout);
//                break;
//            case nets1DropSampleTimeout:
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(nets1ResetMechanisms);
//                }
//                break;
//            case nets1ResetMechanisms:
//                grabber.grab();
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                arm.setRetract();
//                setPathState(nets1ResetMechanismsTimeout);
//            case nets1ResetMechanismsTimeout:
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(toAscentR);
//                }
//            case toAscentR:
//                follower.followPath(getPath(toAscentR));
//                setPathState(holdAscentR);
//                break;
//            case holdAscentR:
//                if (!follower.isBusy()) {
//                    follower.holdPoint(
//                            new EasyPoint(poses.Ascents.Red.Right,
//                                    new Offsets().remX(vars.Chassis.FRONT_LENGTH)),
//                            poses.Ascents.Red.Right.getHeading()
//                    );
//                    setPathState(ascentRGrabberOut);
//                }
//                break;
//            case ascentRGrabberOut:
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                setPathState(ascentRTimeout);
//                break;
//            case ascentRTimeout:
//                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
//                    setPathState(ascentRExtendArm);
//                }
//                break;
//            case ascentRExtendArm:
//                arm.setExtend();
//                setPathState(ascentRExtendArmTimeout);
//                break;
//            case ascentRExtendArmTimeout:
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(ascentRLowerArm);
//                }
//                break;
//            case ascentRLowerArm:
//                arm.setRotation(PrimaryArm.rotationStates.DOWN, 1.5, false);
//                setPathState(opModeStop);
//                break;
//            case opModeStop:
//                if (arm.getRotationState() == PrimaryArm.rotationStates.STOPPED) {
//                    requestOpModeStop();
//                }
//                break;
//        }
//    }
//
//    public void setPathState(PathStates state) {
//        pathState = state;
//        pathTimer.resetTimer();
//        autonomousPathUpdate();
//    }
//}
