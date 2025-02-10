package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CustomPID;

import java.util.PriorityQueue;

/**
 * Manages the DriveTrain of the robot
 * Assumes 4-Wheel Mecanum, has support for both field centric drive and robot centric
 * PreReqs include the CustomPID class, also requires the names on the robot controller to match those in init method
 *
 */
public class DriveTrain {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private double speed = .8;
    private double timer;
    private double xyGradAverage;
    private PriorityQueue<Double> xyGradients = new PriorityQueue<>();
    private double oldX;
    private double oldY;
    private BNO055IMU imu;
    private DcMotor xOdom;
    private DcMotor yOdom;
    HardwareMap hwMap;

    public void init(HardwareMap hwMap){
        this.hwMap = hwMap;

        this.fl = hwMap.get(DcMotor.class, "fl");
        this.fr = hwMap.get(DcMotor.class, "fr");
        this.bl = hwMap.get(DcMotor.class, "bl");
        this.br = hwMap.get(DcMotor.class, "br");
        this.imu = hwMap.get(BNO055IMU.class, "imu");
        this.xOdom = hwMap.get(DcMotor.class, "odo1");
        this.yOdom = hwMap.get(DcMotor.class, "odo2");

        this.xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.xOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.fl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bl.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);

    }
    public void reInitFieldCentric(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);
    }
    public void robotCentricDrive(double leftStickY, double leftStickX, double rightStickX){
        double theta = Math.atan2(-1 * leftStickY, leftStickX);
        double power = Math.hypot(leftStickX, -1 * leftStickY);
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        this.fl.setPower(power * cos/max + rightStickX);
        this.fr.setPower(power * sin/max - rightStickX);
        this.bl.setPower(power * sin/max + rightStickX);
        this.br.setPower(power * cos/max - rightStickX);
    }
    public void fieldCentricDrive(double leftStickY, double leftStickX, double rightStickX, double slow, LinearLift lift){
        double y = -1 * leftStickY; // Remember, this is reversed!
        double x = leftStickX * 1.1; // Counteract imperfect strafing

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rightStickX), 1);
        double frontLeftPower = (rotY + rotX + rightStickX) / (denominator * slow);
        double backLeftPower = (rotY - rotX + rightStickX) / (denominator * slow);
        double frontRightPower = (rotY - rotX - rightStickX) / (denominator * slow);
        double backRightPower = (rotY + rotX - rightStickX) / (denominator * slow);

        //Drive slower if the lift is extended.
        double liftPowerScaling = Math.min(Math.pow(lift.pPower(), 1.5) + 0.3, 1.0);
        double finalSpeed = Math.max(this.speed*liftPowerScaling, 0.15);

        this.fl.setPower(frontLeftPower*this.speed*liftPowerScaling);
        this.fr.setPower(frontRightPower*this.speed*liftPowerScaling);
        this.bl.setPower(backLeftPower*this.speed*liftPowerScaling);
        this.br.setPower(backRightPower*this.speed*liftPowerScaling);

        // Accel
//        if (Math.abs(leftStickX) > 0.1 || Math.abs(leftStickY) > 0.1){
//            double timepassed = (System.nanoTime() - timer) / 1e9;
//
//            if (timepassed < .15){
//                this.incSpeed(.015);
//            }else{
//                this.incSpeed(-0.03);
//            }
//
//            timer = System.nanoTime();
//        }
    }

    public void resetOdoms(){
        this.xOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.xOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.yOdom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.timer = System.nanoTime();
        xyGradients.clear();
    }

    //Helper method for checking Robot movement
    public double delF(){
        double xDiff = (xOdom.getCurrentPosition() - oldX);
        double yDiff = (yOdom.getCurrentPosition() - oldY);

        double magnitude = Math.hypot(xDiff, yDiff);

        oldX = xOdom.getCurrentPosition(); //current x
        oldY = yOdom.getCurrentPosition(); //current y

        return magnitude;
    }

    ///Moves the Robot in some direction + rotates the robot to face some given orientation while driving
    public boolean moveRobot(double[] PidConstantsMove, double[] PidConstantsRot, double driveAngle, double distance, double finalOrientation){
        //Setup PIDs
        driveAngle = Math.toRadians(driveAngle);
        CustomPID distanceControl = new CustomPID(PidConstantsMove);
        CustomPID angleControl = new CustomPID(PidConstantsRot);
        distanceControl.setSetpoint(distance);
        angleControl.setSetpoint(angleWrap(Math.toRadians(finalOrientation)));
        double drivePower = distanceControl.calculateGivenRaw(Math.hypot(xOdom.getCurrentPosition(), yOdom.getCurrentPosition()))[0];
        double rotPower = angleControl.calculateGivenError(angleWrap(Math.toRadians(finalOrientation)-imu.getAngularOrientation().firstAngle))[0];

        moveInDirection(driveAngle, drivePower, rotPower);

        //return true so long as SOMETHING is happening || robot has just started moving.
        return delF() > 5.0 || Math.abs(drivePower) > 0.2 || Math.abs(rotPower) > 0.2 || ((System.nanoTime()-timer)/1e6) < 200;
    }

    public void zeroMotors(){
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveInDirection(0, 0, 0);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2*Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    ///Moves robot in some Direction given a motor power (0-1) and an angle theta in radians (polar coords)
    public void moveInDirection(double theta, double power, double rotPower){
        //Account for [idk] degree offset (Motor powers are weird, IDK)
        theta += 45./180*Math.PI;

        //Convert to motor powers
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        this.fl.setPower(power * cos - rotPower);
        this.fr.setPower(power * sin + rotPower);
        this.bl.setPower(power * sin - rotPower);
        this.br.setPower(power * cos + rotPower);
    }

    public void incSpeed(double inc){
        this.speed = Math.min(Math.max((this.speed+inc), 0.3), 0.5);
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public DcMotor getxOdom() {
        return xOdom;
    }

    public DcMotor getyOdom() {
        return yOdom;
    }

    public double getSpeed(){return this.speed;}
    public double getxPos(){return this.xOdom.getCurrentPosition();}
    public double getyPos(){return this.yOdom.getCurrentPosition();}

    public double getXyGradAverage(){return xyGradAverage;}
}
