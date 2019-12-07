package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TotalControlTeleOp", group = "totalcontrol")
public class TotalControlTeleOp extends OpMode {

    HardwareName robot = new HardwareName(DcMotor.RunMode.RUN_USING_ENCODER);
    double topLeft = 0;
    double topRight = 0;
    double bottomLeft = 0;
    double bottomRight = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper && gamepad1.left_trigger > 0) {
            robot.strafeDiagonal("SW", 1);
        } else if (gamepad1.left_bumper && gamepad1.right_trigger > 0) {
            robot.strafeDiagonal("NW", 1);
        } else if (gamepad1.right_bumper && gamepad1.right_trigger > 0) {
            robot.strafeDiagonal("NE", 1);
        } else if (gamepad1.right_bumper && gamepad1.left_trigger > 0) {
            robot.strafeDiagonal("SE", 1);
        } else if (gamepad1.right_bumper) {
            robot.strafeRight(1);
        } else if (gamepad1.left_bumper) {
            robot.strafeLeft(1);
        } else if (gamepad1.left_stick_x != 0 && gamepad1.right_trigger > 0) {
            robot.turn(gamepad1.right_trigger, gamepad1.left_stick_x);
        } else if (gamepad1.left_stick_x != 0 && gamepad1.left_trigger > 0) {
            robot.turn(-gamepad1.left_trigger, gamepad1.left_stick_x);
        } else if (gamepad1.left_stick_x > 0) {
            robot.pivotRight(1);
        } else if (gamepad1.left_stick_x < 0) {
            robot.pivotLeft(1);
        } else if (gamepad1.right_trigger > 0) {
            robot.moveForward(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            robot.moveBackward(gamepad1.left_trigger);
        } else {
            robot.stopMove();
        }

        if (gamepad2.right_trigger != 0) {
            robot.intake(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger != 0) {
            robot.release(gamepad2.left_trigger);
        } else if (gamepad2.right_bumper) {
            robot.rotateArm(0.5);
        } else if (gamepad2.left_bumper) {
            robot.rotateArm(-0.5);
        } else if (gamepad2.left_stick_y < 0) {
            robot.slideArm(-(gamepad2.left_stick_y));
        } else if (gamepad2.left_stick_y > 0) {
            robot.slideArm(gamepad2.left_stick_y);
        } else {
            robot.stopArm();

            telemetry.addData("Top Left Drive", robot.topLeftDrive.getPower());
            telemetry.addData("Bottom Left Drive", robot.bottomLeftDrive.getPower());
            telemetry.addData("Top Right Drive", robot.topRightDrive.getPower());
            telemetry.addData("Bottom Right Drive", robot.bottomRightDrive.getPower());
            telemetry.addData("Grabber Rotational",robot.armRotate.getCurrentPosition());
            telemetry.addData("Grabber Linear",robot.armLinear.getCurrentPosition());
            telemetry.update();
        }

    }

}
