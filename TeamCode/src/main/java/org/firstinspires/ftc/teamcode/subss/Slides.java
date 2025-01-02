package org.firstinspires.ftc.teamcode.subss;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Slides {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    public Encoder leftSlideEncoder;
    public Encoder rightSlideEncoder;

    OpMode opMode;

    int velocity = 5000; //4250

    public Slides(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        leftSlide = opMode.hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = opMode.hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlideEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "leftSlide"));
        rightSlideEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rightSlide"));
        rightSlideEncoder.setDirection(Encoder.Direction.REVERSE);


    }

    public void pickupSample(int slidesPosition) {
        leftSlide.setTargetPosition(slidesPosition);
        rightSlide.setTargetPosition(slidesPosition);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void placeSample() {
        leftSlide.setTargetPosition(2125);
        rightSlide.setTargetPosition(2125);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void placeSpecimen() {
        leftSlide.setTargetPosition(900);
        rightSlide.setTargetPosition(900);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void moveToResetPos() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void moveSlidesManual(double joystick) {
        leftSlide.setVelocity(joystick * 1000);
        rightSlide.setVelocity(joystick * 1000);
    }


    public int leftGetCurrentPosition() {
        return leftSlide.getCurrentPosition();
    }

    public int rightGetCurrentPosition() {
        return rightSlide.getCurrentPosition();
    }


    public void resetEncoder() {
        leftSlideEncoder.reset();
        rightSlideEncoder.reset();
    }
}
