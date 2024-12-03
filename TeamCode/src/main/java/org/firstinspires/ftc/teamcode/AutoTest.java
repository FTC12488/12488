package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.robotParts.Claw;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;
import org.firstinspires.ftc.teamcode.robotParts.Turn;

@Autonomous(name="AutoTest", group="Linear OpMode")
@Config
public class AutoTest extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double[] PidConstantsAngle = new double[]{1, 200, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.01, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
//    private final Claw claw = new Claw();

    public static String[][] instructions = {
            //Move forward
            {"Drive", "0", "8000"},
            {"Turn", "180"},
            {"Lift", "600"},
            {"Drive", "180", "1000"},
            {"Lift", "0"},
            {"Turn", "180"},
            {"Drive", "135", "10000"},
//
//            //Place
//            {"Lift", "400"},
//
//            //Back up
//            {"Drive", "180", "2000"},
    };

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        turn.init(hardwareMap);
//        claw.init(hardwareMap);

        waitForStart();

        for(String[] instruction : instructions){
            double theta = 0;
            double distance = 0;
            double timeout = System.nanoTime();
            switch (instruction[0]) {
                case "Drive":
                    double oldAngle = dt.getImu().getAngularOrientation().firstAngle;
                    theta = Double.parseDouble(instruction[1]);
                    distance = Double.parseDouble(instruction[2]);
                    dt.driveToLocation(PidConstantsDistance, theta, distance);

                    //Fix Angle
                    timeout = System.nanoTime(); //Make sure it's basing the timeout on time passed since robot reached target loc
                    theta = dt.getImu().getAngularOrientation().firstAngle-oldAngle;
                    while(Math.abs((Math.toRadians(theta)-dt.getImu().getAngularOrientation().firstAngle)) > .02 && opModeIsActive()){
                        if((System.nanoTime() - timeout)/1e9 > 1){
                            break;
                        }
                        dt.fixAngle(PidConstantsAngle, theta);
                        packet.put("Angle", dt.getImu().getAngularOrientation().firstAngle);
                        dashboard.sendTelemetryPacket(packet);
                    }
                    dt.zeroMotors();
                    break;
                case "Turn":
                    theta = Double.parseDouble(instruction[1]);
                    while(Math.abs((Math.toRadians(theta)-dt.getImu().getAngularOrientation().firstAngle)) > .001 && opModeIsActive()){
                        if((System.nanoTime() - timeout)/1e9 > 3){
                            break;
                        }
                        dt.fixAngle(PidConstantsAngle, theta);
                        packet.put("Angle", dt.getImu().getAngularOrientation().firstAngle);
                        dashboard.sendTelemetryPacket(packet);
                    }
                    dt.zeroMotors();
                    break;
                case "Lift":
                    distance = Double.parseDouble(instruction[1]);
                    while(Math.abs(distance-lin.getPos()) > 45){
                        lin.gotoPosition(distance, new CustomPID(PidConstantsAngle));
                        packet.put("Lin", lin.getPos());
                        dashboard.sendTelemetryPacket(packet);
                    }
                    break;
                case "DLift":
                    theta = Double.parseDouble(instruction[1]);
                    turn.gotoMaxPosition(theta);
                    break;
//                case "Rotate":
//                    theta = Double.parseDouble(instruction[1]);
//                    claw.setRotate(theta);
//                    break;
//                case "Claw":
//                    if (instruction[1].equals("Close")){
//                        claw.setClaw(.8);
//                    }else{
//                        claw.setClaw(.5);
//                    }
//                    break;
            }
        }
    }
}
