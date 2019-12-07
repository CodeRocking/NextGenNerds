package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareName {
    public DcMotor bottomLeftDrive;
    public DcMotor bottomRightDrive;
    public DcMotor topLeftDrive;
    public DcMotor topRightDrive;
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public DcMotor armRotate;
    public DcMotor armLinear;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public Servo pusher;
    public Orientation angle;

    private DcMotor.RunMode initialMode;

    HardwareMap map;

    public HardwareName(DcMotor.RunMode enteredMode) {
        initialMode = enteredMode;
    }

    public void init(HardwareMap aMap) /*throws InterruptedException*/ {
        map = aMap;

        bottomLeftDrive = map.dcMotor.get("bottomLeftDrive");
        bottomRightDrive = map.dcMotor.get("bottomRightDrive");
        topLeftDrive = map.dcMotor.get("topLeftDrive");
        topRightDrive = map.dcMotor.get("topRightDrive");
        leftIntake = map.dcMotor.get("leftIntake");
        rightIntake = map.dcMotor.get("rightIntake");
        armRotate = map.dcMotor.get("armRotate");
        armLinear = map.dcMotor.get("armLinear");
        leftGrabber = map.servo.get("leftGrabber");
        rightGrabber = map.servo.get("rightGrabber");
        pusher = map.servo.get("pusher");

        //Encoders
        bottomLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set mode
        bottomLeftDrive.setMode(initialMode);
        bottomRightDrive.setMode(initialMode);
        topLeftDrive.setMode(initialMode);
        topRightDrive.setMode(initialMode);
        leftIntake.setMode(initialMode);
        rightIntake.setMode(initialMode);
        armRotate.setMode(initialMode);
        armLinear.setMode(initialMode);

        //set zero power mode
        bottomLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        //set direction
        bottomLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        topRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stopMove() {
        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);
    }

    public void stopArm() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        armLinear.setPower(0);
        armRotate.setPower(0);
    }


    public void strafeLeft(double power) {
        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(-power);
        topRightDrive.setPower(power);
    }

    public void strafeRight(double power) {
        bottomLeftDrive.setPower(-power);
        bottomRightDrive.setPower(power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(-power);
    }

    public void strafeDiagonal(String direction, float power) {
        switch (direction) {
            case "NE":
                bottomLeftDrive.setPower(0);
                bottomRightDrive.setPower(power);
                topLeftDrive.setPower(power);
                topRightDrive.setPower(0);
                break;
            case "SE":
                bottomLeftDrive.setPower(-power);
                bottomRightDrive.setPower(0);
                topLeftDrive.setPower(0);
                topRightDrive.setPower(-power);
                break;

            case "SW":
                bottomLeftDrive.setPower(0);
                bottomRightDrive.setPower(-power);
                topLeftDrive.setPower(-power);
                topRightDrive.setPower(0);
                break;
            case "NW":
                bottomLeftDrive.setPower(power);
                bottomRightDrive.setPower(0);
                topLeftDrive.setPower(0);
                topRightDrive.setPower(power);
                break;
        }
    }

    public void moveForward(float power) {
        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(power);
    }

    public void moveBackward(float power) {
        bottomLeftDrive.setPower(-power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(-power);
        topRightDrive.setPower(-power);
    }

    public void pivotRight(float power) {
        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(-power);
    }

    public void pivotLeft(float power) {
        bottomLeftDrive.setPower(-power);
        bottomRightDrive.setPower(power);
        topLeftDrive.setPower(-power);
        topRightDrive.setPower(power);
    }

    public void turn(float power, float turn) {
        power = power / 2;
        turn = turn / 2;

        bottomLeftDrive.setPower(power + turn);
        bottomRightDrive.setPower(power - turn);
        topLeftDrive.setPower(power + turn);
        topRightDrive.setPower(power - turn);
    }

    public void betterTurn(float turn) {
        if (turn < 0) {
            topLeftDrive.setPower(0);
            bottomLeftDrive.setPower(0);
        } else if (turn > 0) {
            topRightDrive.setPower(0);
            bottomRightDrive.setPower(0);
        } else {
            topLeftDrive.setPower(1);
            bottomLeftDrive.setPower(1);
            topRightDrive.setPower(1);
            bottomRightDrive.setPower(1);
        }
    }

    public void intake(float power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void release(float power) {
        leftIntake.setPower(-power);
        rightIntake.setPower(-power);
    }

    public void rotateArm(double power) {
        armRotate.setPower(power);
    }

    public void slideArm(double power) {
        armLinear.setPower(power);
    }


}

