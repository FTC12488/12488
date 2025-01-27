package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.Claw;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;
import org.firstinspires.ftc.teamcode.robotParts.Turn;


@TeleOp(name="DriveTrainTest", group="Linear OpMode")
@Config
public class DriveTrainTest extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double distance;
    public static double targetTurn = 1.05;
    public static double targetRotateUp = 0.5;
    public static double targetRotateDown = 0.61;
    public static double targetRotatePlace = 0.3;
    private final double PLACEANGLE = -30;
    private final double GRABANGLE = -90;
    public static double[] PidConstantsAngle = new double[]{1, 200, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.01, 0};
    public static double[] PidConstantsLift = new double[]{.25, 25, 0.009};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        turn.init(hardwareMap);
        claw.init(hardwareMap);
//        boolean toggle = true;
        waitForStart();

        claw.setRotate(.5);
        claw.setClaw(.5);

        while (opModeIsActive()) {
//            if(true){
//                dt.fieldCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
//            }else{
//                dt.robotCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
//            }
//            if(gamepad1.start){
//                toggle = !toggle;
//            }
            //Drive Train
            dt.fieldCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1, lin);
            if(gamepad1.x){
                dt.fixAngle(PidConstantsAngle, GRABANGLE);
            }
            if(gamepad1.y){
                dt.fixAngle(PidConstantsAngle, PLACEANGLE);
            }
            if(gamepad1.dpad_left){
                dt.reInitFieldCentric();
            }
            //Speed control
            if(gamepad1.right_bumper){
                dt.incSpeed(0.4);
            }
            if(gamepad1.left_bumper){
                dt.incSpeed(-0.4);
            }
            //Lift lift
            if(gamepad2.left_stick_y!=0){
                if (Math.abs(turn.getPos()) > 0.1) {
                    lin.setMAX(600);
                }else{
                    lin.setMAX(-1);
                }
                lin.moveLift(gamepad2.left_stick_y);
            }else{
                lin.setPower(0);
            }
            //Turn lift
            if(gamepad2.right_stick_y!=0 && (lin.getPos() <= 300) || gamepad2.dpad_down){
                turn.turn(gamepad2.right_stick_y);
            }else{
                turn.setPower(0);
            }
            //Claw
            if (gamepad2.right_trigger != 0) {
                claw.setRotate(claw.getRotate()+0.01);
            }
            if (gamepad2.left_trigger != 0) {
                claw.setRotate(claw.getRotate()-0.01);
            }
            if (gamepad2.dpad_up){
                claw.setRotate(0.151);
            }
            //Claw Open/close
            if (gamepad2.left_bumper) {
                claw.setClaw(.5); //.5
//                claw.setPower(.5);
            } else if (gamepad2.right_bumper) {
                claw.setClaw(.24); //.24
//                claw.setPower(-.5);
//            }else{
//                claw.setPower(0);
            }
            //Reinit Operator
            if(gamepad2.dpad_left){
                lin.reInit();
                turn.reInit();
            }
            //Presets
            if(gamepad2.x){
                if((lin.getPos() < 600)){
                    turn.gotoMaxPosition(targetTurn);
                }
                claw.setRotate(targetRotateUp);
            }
            if(gamepad2.a){
                claw.setRotate(targetRotateDown);
            }
            if(gamepad2.y){
                if((lin.getPos() < 300)){
                    turn.gotoMaxPosition(0);
                }else{
                    claw.setRotate(targetRotatePlace);
                    claw.setClaw(.5);
                }
            }
            if(gamepad2.b){
                dt.incSpeed(-0.9);
            }
//            packet.put("Angle", angle);
            packet.put("X", dt.getxOdom().getCurrentPosition());
            packet.put("Y", dt.getyOdom().getCurrentPosition());
            packet.put("Lin", lin.getPos());
            packet.put("Turn", turn.getPos());
            packet.put("Speed", dt.getSpeed());
            packet.put("Claw Rotation", claw.getRotate());
            packet.put("Claw Open", claw.getPos());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}

