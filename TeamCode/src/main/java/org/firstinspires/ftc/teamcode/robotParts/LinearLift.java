package org.firstinspires.ftc.teamcode.robotParts;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomPID;

public class LinearLift {
    private DcMotor lift;
    private final double INCHTOENCH = 115.35;

    private final double LINMAX = 2150;

    public void init(HardwareMap hwMap){
        this.lift = hwMap.get(DcMotor.class, "lift");
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void reInit(){
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        this.lift.setPower(power);
    }
    public double getPos(){
        return this.lift.getCurrentPosition();
    }
    public void gotoPosition(double positionInches, CustomPID pid){
        pid.setSetpoint(positionInches);
        if (Math.abs((positionInches-this.lift.getCurrentPosition())) > 45){
            double[] outputs = pid.calculateGivenRaw(this.lift.getCurrentPosition());
            double power = outputs[0];
            this.lift.setPower(power);
        }else{
            this.lift.setPower(0.0);
        }
    }
    //Percentage power, calculated based on how far we are from the max position, further = more power
    public double pPower(){
        return (1-Math.abs(getPos()/LINMAX));
    }
    public void moveLift(double gamepadInput){
        if (Math.abs(this.lift.getCurrentPosition()) < LINMAX) {
            //Percentage power, calculated based on how far we are from the max position, further = more power
            double p = pPower();
            //Constant power, added to input in order to counteract gravity
            double c = Math.abs(getPos()/LINMAX * 0.1);
            if(gamepadInput > 0){
                p = .4;
            }
            this.lift.setPower((-p * gamepadInput) + c);
        } else {
            this.lift.setPower(0);
        }
    }
    public DcMotor getLift() {
        return this.lift;
    }
}
