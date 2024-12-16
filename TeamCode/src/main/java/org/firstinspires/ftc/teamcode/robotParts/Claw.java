package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CustomPID;

public class Claw {
    private Servo claw;
    private Servo rotate;
    private Servo intk;

    public void init(HardwareMap hwMap){
        this.claw = hwMap.get(Servo.class, "claw");
        this.claw.setDirection(Servo.Direction.REVERSE);
        this.rotate = hwMap.get(Servo.class, "rotate");
        this.rotate.setDirection(Servo.Direction.REVERSE);
        this.intk = hwMap.get(CRServo.class, "intk");
        this.intk.setDirection(CRServo.Direction.REVERSE);
    }

    public void setClaw(double pos) {
        this.claw.setPosition(pos);
    }
    public void setIntk(double pow) {
        this.intk.setPower(pow);
    }
    public void setRotate(double pos){
        this.rotate.setPosition(pos);
    }


    public double getPos(){return this.claw.getPosition();}
    public double getRotate(){return this.rotate.getPosition();}
}
