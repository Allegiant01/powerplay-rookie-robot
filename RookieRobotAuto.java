
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Rookie Robot Auto")
public class RookieRobotAuto extends LinearOpMode {
    double UNCLAMPED = 0.25, CLAMPED = 0;
    DcMotor fl, bl, fr, br, left_slide, right_slide;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");
        left_slide = hardwareMap.get(DcMotor.class, "left_intake");
        right_slide = hardwareMap.get(DcMotor.class, "right_intake");
        claw = hardwareMap.get(Servo.class, "clamp_servo");
        claw.setPosition(CLAMPED);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        left_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            moveMecanum(12, 0, 0, 0.5);
            Thread.sleep(2000);
            moveMecanum(12, 0, 0, -0.5);
            Thread.sleep(2000);
            moveMecanum(0, 12, 0, 0.5);
            Thread.sleep(2000);
            moveMecanum(0, 12, 0, -0.5);
            Thread.sleep(2000);
            moveMecanum(0, 0, 12, 0.5);
            Thread.sleep(2000);
            moveMecanum(0, 0, 12, -0.5);
        }
    }

    void moveMecanum(double forward, double strafe, double rotate, double power) {
        int fl_init = fl.getCurrentPosition();
        int fr_init = fr.getCurrentPosition();
        int bl_init = bl.getCurrentPosition();
        int br_init = br.getCurrentPosition();

        int fl_pos = fl.getCurrentPosition();
        int fr_pos = fr.getCurrentPosition();
        int bl_pos = bl.getCurrentPosition();
        int br_pos = br.getCurrentPosition();

        if (forward != 0) {
            while (Math.abs(fl_pos - fl_init) < forward * 31 && Math.abs(fr_pos - fr_init) < forward * 31 && Math.abs(bl_pos - bl_init) < forward * 31 && Math.abs(br_pos - br_init) < forward * 31) {
                fl.setPower(power);
                bl.setPower(power);
                fr.setPower(power);
                br.setPower(power);

                fl_pos = fl.getCurrentPosition();
                fr_pos = fr.getCurrentPosition();
                bl_pos = bl.getCurrentPosition();
                br_pos = br.getCurrentPosition();
            }
        } else if (strafe != 0) {
            while (Math.abs(fl_pos - fl_init) < strafe * 62 && Math.abs(fr_pos - fr_init) < strafe * 62 && Math.abs(bl_pos - bl_init) < strafe * 62 && Math.abs(br_pos - br_init) < strafe * 62) {
                fl.setPower(power);
                bl.setPower(-power);
                fr.setPower(-power);
                br.setPower(power);

                fl_pos = fl.getCurrentPosition();
                fr_pos = fr.getCurrentPosition();
                bl_pos = bl.getCurrentPosition();
                br_pos = br.getCurrentPosition();
            }
        } else if (rotate != 0) {
            while (Math.abs(fl_pos - fl_init) < rotate * 124 && Math.abs(fr_pos - fr_init) < rotate * 124 && Math.abs(bl_pos - bl_init) < rotate * 124 && Math.abs(br_pos - br_init) < rotate * 124) {
                fl.setPower(power);
                bl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);

                fl_pos = fl.getCurrentPosition();
                fr_pos = fr.getCurrentPosition();
                bl_pos = bl.getCurrentPosition();
                br_pos = br.getCurrentPosition();
            }
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        left_slide.setTargetPosition(1500);
        right_slide.setTargetPosition(1500);
        right_slide.setPower(0.7);
        left_slide.setPower(0.7);
        left_slide.setMode(RUN_TO_POSITION);
        right_slide.setMode(RUN_TO_POSITION);

        claw.setPosition(UNCLAMPED);
        //need to add smth like thread.sleep between these 2
        claw.setPosition(CLAMPED);

        left_slide.setTargetPosition(0);
        right_slide.setTargetPosition(0);
        left_slide.setPower(0.7);
        right_slide.setPower(0.7);
        left_slide.setMode(RUN_TO_POSITION);
        right_slide.setMode(RUN_TO_POSITION);
    }
}