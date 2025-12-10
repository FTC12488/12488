package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotParts.Claw;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain1;
//import org.firstinspires.ftc.teamcode.robotParts.Launcher;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;
import org.firstinspires.ftc.teamcode.robotParts.Turn;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DriveFinal", group="Linear OpMode")
@Config
public class DriveTrainLauncherFinal extends LinearOpMode {
    private CRServo servoL;
    private CRServo servoR;
    private double power=0;

    private Servo push;
    private double sped;
    private DcMotor motorL;
    private DcMotor motorR;
    private DcMotor intake;
    private double strength;
    private CRServo spinnerL;
    private double launchstrength;
    private double intakestrength;







    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private final double PLACEANGLE = -30;
    private final double GRABANGLE = -90;
    public static double[] PidConstantsAngle = new double[]{1, 200, 0};



    private final DriveTrain1 dt = new DriveTrain1();
    //private final Launcher la = new Launcher();


    @Override
    public void runOpMode() {
        servoL = hardwareMap.crservo.get("servoL");
        servoR = hardwareMap.crservo.get("servoR");
        spinnerL = hardwareMap.crservo.get("spinnerL");
        push = hardwareMap.servo.get("push");
        motorL = hardwareMap.dcMotor.get("right");
        motorR = hardwareMap.dcMotor.get("left");
        intake = hardwareMap.dcMotor.get("intake");

        strength=-0.4; // intake strength
        intakestrength=strength;
        launchstrength=0.36;
        sped=0.25; //servo speed
        push.setPosition(0.05);



        dt.init(hardwareMap);

//        boolean toggle = true;
        waitForStart();



        while (opModeIsActive()) {

            //Drive Train
            dt.fieldCentricDrive(-(gamepad1.left_stick_y), -(gamepad1.left_stick_x), (gamepad1.right_stick_x), 1);
            // x and y are basically useless for the 2025-2026 season
//            if(gamepad1.x){
//                dt.fixAngle(PidConstantsAngle, GRABANGLE);
//            }
//            if(gamepad1.y){
//                dt.fixAngle(PidConstantsAngle, PLACEANGLE);
//            }


            //Movement
//            if(gamepad1.x){
//                dt.fixAngle(PidConstantsAngle, GRABANGLE);
//            }
//            if(gamepad1.y){
//                dt.fixAngle(PidConstantsAngle, PLACEANGLE);
//            }
            if(gamepad1.dpad_left){
                dt.reInitFieldCentric();
            }
            //Speed control
            if(gamepad1.right_bumper){
                dt.incSpeed(1);
            }
            if(gamepad1.left_bumper){
                dt.incSpeed(-0.4);
            }


            //servos
            if (gamepad2.b) {
//                push.setPosition(0.2);
                power = -2*sped;
            }
            else if (gamepad2.a){
//                push.setPosition(0.05);
                power=sped;
            }
            else if (gamepad2.x){
                power=-sped;
            }

            else if (gamepad2.y){
                power=0;
            }

            servoL.setPower(-power);
            servoR.setPower(power);
            spinnerL.setPower(-power);

            //Launcher
            if (gamepad2.left_bumper){
                motorL.setPower(launchstrength);
                motorR.setPower(-launchstrength);
            }

            if (gamepad2.right_bumper){
                motorL.setPower(0);
                motorR.setPower(0);
            }


            //intake


            //Actual intake
            if (gamepad2.left_trigger>0){
                intake.setPower(intakestrength);
            }
            else if (gamepad2.right_trigger>0){
                intake.setPower(2*intakestrength);
            }
            else{
                intake.setPower(0);
            }





//            packet.put("X", dt.getxOdom().getCurrentPosition());
//            packet.put("Y", dt.getyOdom().getCurrentPosition());

            packet.put("Speed", dt.getSpeed());


            dashboard.sendTelemetryPacket(packet);
        }
    }
}

