package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {

    private DcMotor lfDrive, lbDrive, rfDrive, rbDrive;
    private Servo pivot;

    private HardwareMap hwMap;
    OpMode opMode;

    private static final double WIDTH = 16;
    private static final double LENGTH = 5;
    private static final double MAX_SPEED = 1.0;




    public void init(HardwareMap ahwMap, OpMode op) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        opMode = op;

        this.pivot = hwMap.get(Servo.class, "pivot");

        // Define and Initialize Motors
        this.lfDrive  = hwMap.get(DcMotor.class, "lfDrive");
        this.lbDrive  = hwMap.get(DcMotor.class, "lbDrive");
        this.rfDrive = hwMap.get(DcMotor.class, "rfDrive");
        this.rbDrive = hwMap.get(DcMotor.class, "rbDrive");
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection((DcMotor.Direction.REVERSE));
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        rbDrive.setDirection((DcMotor.Direction.FORWARD));

        stop();

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }



    public void setPivotPosition(double pivotPosition){
        pivot.setPosition(pivotPosition);
    }

    public void skidSteerDrive(double lPower, double rPower){
        lfDrive.setPower(lPower);
        lbDrive.setPower(lPower);
        rfDrive.setPower(rPower);
        rbDrive.setPower(rPower);
    }

    // Moves the drive train using the given x, y, and rotational velocities
    public void mechanumDrive(double xVelocity, double yVelocity, double wVelocity) {
        double speedScale = 1.0;

        double rfVelocity = yVelocity - xVelocity + wVelocity * (WIDTH / 2 + LENGTH / 2);
        double lfVelocity = yVelocity + xVelocity - wVelocity * (WIDTH / 2 + LENGTH / 2);
        double lbVelocity = yVelocity - xVelocity - wVelocity * (WIDTH / 2 + LENGTH / 2);
        double rbVelocity = yVelocity + xVelocity + wVelocity * (WIDTH / 2 + LENGTH / 2);

        double maxVelocity = findMax(lfVelocity, rfVelocity, lbVelocity, rbVelocity);

        if (maxVelocity > MAX_SPEED) {
            speedScale = MAX_SPEED / maxVelocity;
        }

        lfDrive.setPower(lfVelocity * speedScale);
        rfDrive.setPower(rfVelocity * speedScale);
        lbDrive.setPower(lbVelocity * speedScale);
        rbDrive.setPower(rbVelocity * speedScale);

    }

    // Helper function for power scaling in the drive method
    private double findMax(double... vals) {
        double max = Double.NEGATIVE_INFINITY;

        for (double d : vals) {
            if (d > max) max = d;
        }

        return max;
    }

    public void stop(){
        lfDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        rbDrive.setPower(0);
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        rfDrive.setMode(mode);
        rbDrive.setMode(mode);
        lfDrive.setMode(mode);
        lbDrive.setMode(mode);
    }
}