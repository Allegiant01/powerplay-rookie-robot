package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FieldCentricMecanum extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double UNCLAMPED = 0.25, CLAMPED = 0;

        DcMotor fLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor fRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor bLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor bRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        Servo claw = hardwareMap.get(Servo.class, "clamp_servo");
        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "left_intake");
        DcMotor linearSlide2 = hardwareMap.get(DcMotor.class, "right_intake");
        Servo rotate = hardwareMap.get(Servo.class, "rotate_servo");

        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide2.setDirection(DcMotor.Direction.REVERSE);
        fLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        linearSlide.setMode(STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(STOP_AND_RESET_ENCODER);
        linearSlide.setMode(RUN_USING_ENCODER);
        linearSlide2.setMode(RUN_USING_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated() && !isStopRequested()) {
            Thread.sleep(1);
            telemetry.addData("Calibrating IMU...", imu.isGyroCalibrated());
            telemetry.update();
        }
        telemetry.addData("IMU Calibrated", imu.isGyroCalibrated());
        telemetry.update();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double rand = 5;
            telemetry.addData("motor angle", rand);
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double direction = Math.atan2(y, x);
            double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(direction * magnitude), 1);
            double fRightPower = (Math.sin(direction - (Math.PI)/4) * magnitude)/denominator;
            double fLeftPower = (Math.sin(direction + (Math.PI)/4) * magnitude)/denominator;

            fLeftMotor.setPower(fLeftPower);
            bLeftMotor.setPower(fRightPower);
            fRightMotor.setPower(fRightPower);
            bRightMotor.setPower(fLeftPower);

             if (gamepad1.right_stick_x != 0) {
                    fLeftMotor.setPower(turn);
                    bLeftMotor.setPower(turn);
                    fRightMotor.setPower(-turn);
                    bRightMotor.setPower(-turn);
                }
             //heights for linear slide are for medium junctions
                if(gamepad1.left_bumper) {
                    linearSlide.setTargetPosition(1600);
                    linearSlide2.setTargetPosition(1600);
                    linearSlide2.setPower(0.7);
                    linearSlide.setPower(0.7);
                    linearSlide2.setMode(RUN_TO_POSITION);
                    linearSlide.setMode(RUN_TO_POSITION);
                }
                if(gamepad1.right_bumper) {
                    linearSlide.setTargetPosition(0);
                    linearSlide2.setTargetPosition(0);
                    linearSlide.setPower(1);
                    linearSlide2.setPower(1);
                    linearSlide.setMode(RUN_TO_POSITION);
                    linearSlide2.setMode(RUN_TO_POSITION);
                }
                if(gamepad1.a) {
                    claw.setPosition(UNCLAMPED);
                }
                if(gamepad1.y) {
                    claw.setPosition(CLAMPED);
                }
                if(gamepad1.x) {
                    rotate.setPosition(0.80);
                }
                if(gamepad1.b) {
                    rotate.setPosition(1);
                }
        }
    }
}
