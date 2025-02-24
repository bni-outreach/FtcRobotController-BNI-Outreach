package org.firstinspires.ftc.teamcode.Outreach.Robots;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "Rapid Revolvers")
public class rapidRevolvers extends LinearOpMode {
    // Before running, we will need to make sure the control hub and driver station is in the correct version

    private DcMotor mainMotor;
    private double targetRotations = 5;
    private final double encoderCountsPerRotation = 1440;
    private int targetPosition = (int) (targetRotations * encoderCountsPerRotation);

    public void initRobot(HardwareMap hwMap) {

        mainMotor = hwMap.dcMotor.get("mainMotor");

        mainMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mainMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void stopMotors() {
        mainMotor.setPower(0);
        mainMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() {
        initRobot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double leftStickYVal = -gamepad1.left_stick_y;
            mainMotor.setPower(leftStickYVal);

            if (gamepad1.x) {
                mainMotor.setTargetPosition(1000);
                int targetPos = mainMotor.getCurrentPosition() + targetPosition;
                mainMotor.setTargetPosition(targetPos);

                mainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mainMotor.setPower(0.5);

                while (opModeIsActive() && mainMotor.isBusy()) {
                    telemetry.addData("Current Position", mainMotor.getCurrentPosition());
                    telemetry.addData("Target Position", targetPos);
                    telemetry.addData("Motor Power", mainMotor.getPower());
                    telemetry.addData("Is Motor Busy", mainMotor.isBusy());
                    telemetry.update();

                    if (gamepad1.a) {
                        stopMotors();
                        telemetry.addData("A Pressed", "Stopping Motor");
                        telemetry.update();
                        break;
                    }
                }
                mainMotor.setPower(0);
                mainMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.a) {
                stopMotors();
            }
            telemetry.addData("Motor Power", mainMotor.getPower());
            telemetry.addData("Motor Position", mainMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}
