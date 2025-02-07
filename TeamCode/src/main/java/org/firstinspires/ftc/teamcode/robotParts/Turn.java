package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomPID;

public class Turn {
    private DcMotor dlift;

    private final double INCHTOENCH = 115.35;
    private final double TURNMAX = 2400;

    public void init(HardwareMap hwMap){
        this.dlift = hwMap.get(DcMotor.class, "dlift");
        this.dlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.dlift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void reInit(){
        this.dlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.dlift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        this.dlift.setPower(power);
    }
    public double getPos(){
        return Math.abs(this.dlift.getCurrentPosition())/TURNMAX;
    }
    public void turn(double gamepadInput){
        if ((Math.abs(this.dlift.getCurrentPosition()) < TURNMAX) || gamepadInput < 0) {
            this.dlift.setPower(-0.35 * gamepadInput);
        } else {
            this.dlift.setPower(0);
        }
    }

    public void getToPos(double percent){
        // Calc power
        CustomPID c1 = new CustomPID(new double[]{.0000012, 0.000035, 0.000015});
        c1.setSetpoint(-TURNMAX * percent);
        double[] outputs = c1.calculateGivenRaw(this.dlift.getCurrentPosition());
        double power = outputs[0];

        // dw about it
        double target = Math.abs(TURNMAX * percent);
        double currentPos = Math.abs(this.dlift.getCurrentPosition());

        // Set power or something
        if ((power > 0 && currentPos > target) || (power < 0 && currentPos < target)){
            this.dlift.setPower(power);
        }else{
            this.dlift.setPower(0.0);
        }
    }

}
