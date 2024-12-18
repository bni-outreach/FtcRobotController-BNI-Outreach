package org.firstinspires.ftc.teamcode.Outreach.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Outreach.Drivetrains.MecanumDrive;

public class SarahDeckerTshirtLauncher extends LinearOpMode {
    // Before running, we will need to make sure the control hub and driver station is in the correct version

    private DcMotor mainMotor = null;

    private double targetRotations =5;
    private final double encoderCountsPerRotation = 1440;


     public void initOpMode (HardwareMap hwMap) {

         mainMotor = hwMap.dcMotor.get("mainMotor");

         mainMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         mainMotor.setDirection(DcMotorSimple.Direction.FORWARD);

         mainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         waitForStart();

     }
     public void runOpMode() {

        while (opModeIsActive()) {
            double leftStickYVal = -gamepad1.left_stick_y;
            mainMotor.setPower(leftStickYVal);

            if (gamepad1.x) {
                double targetPosition = targetRotations * encoderCountsPerRotation;
                mainMotor.setPower(0.5);
                double initialPosition = mainMotor.getCurrentPosition();

                while (opModeIsActive() && (mainMotor.getCurrentPosition() - initialPosition) < targetPosition) {
                    telemetry.addData("Current Position", mainMotor.getCurrentPosition());
                    telemetry.update();
                }
                mainMotor.setPower(0);
            }


        }
     }
}

