package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {
    public DcMotor bottomLeftDrive;
    public DcMotor bottomRightDrive;
    public DcMotor topLeftDrive;
    public DcMotor topRightDrive;
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public DcMotor arm;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public Servo Pusher;
    public Orientation angle;

    private DcMotor.RunMode initialMode;

    HardwareMap map;

    public HardwareStrafer(DcMotor.RunMode enteredMode) {
        initialMode = enteredMode;
    }

    public void init(HardwareMap aMap) /*throws InterruptedException*/{
        map = aMap;

        bottomLeftDrive = map.dcMotor.get("bottomLeftDrive");
        bottomRightDrive = map.dcMotor.get("bottomRightDrive");
        topLeftDrive = map.dcMotor.get("topLeftDrive");
        topRightDrive = map.dcMotor.get("topRightDrive");
        leftIntake = map.dcMotor.get("leftIntake");
        rightIntake = map.dcMotor.get("rightIntake");
        arm = map.dcMotor.get("arm");

        //Encoders
        bottomLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set mode
        bottomLeftDrive.setMode(initialMode);
        bottomRightDrive.setMode(initialMode);
        topLeftDrive.setMode(initialMode);
        topRightDrive.setMode(initialMode);

        //set zero power mode
        bottomLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set direction
        bottomLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        topRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stop() {
        bottomLeftDrive.setPower(0);
        bottomRightDrive.setPower(0);
        topLeftDrive.setPower(0);
        topRightDrive.setPower(0);
    }

    public float getAngle() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    public void turnAngle(float power, int degrees){
        float heading = getAngle();
        float target = heading + degrees;
        while (Math.abs(getAngle()-target)>3) {

            if (getAngle() > target) {
                pivot(power);
            } else if (getAngle() < target) {
                pivot(-power);
            }
        }
        stop();
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

    public void pivot(float power) {
        bottomLeftDrive.setPower(power);
        bottomRightDrive.setPower(-power);
        topLeftDrive.setPower(power);
        topRightDrive.setPower(-power);
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
        }else if (turn > 0) {
            topRightDrive.setPower(0);
            bottomRightDrive.setPower(0);
        } else{
            topLeftDrive.setPower(1);
            bottomLeftDrive.setPower(1);
            topRightDrive.setPower(1);
            bottomRightDrive.setPower(1);
        }
    }

    public void intakeOn(float power) {

    }
}

