package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;
import org.firstinspires.ftc.teamcode.robotParts.Turn;
import org.firstinspires.ftc.teamcode.robotParts.Claw;

@Autonomous(name="LeftAuto", group="Linear OpMode")
@Config
public class LeftAuto extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double[] PidConstantsAngle = new double[]{1, 350, 0};
    public static double[] PidConstantsDistance = new double[]{0.0005, 0.1, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();

    public static String[][] instructions = {
            //Place pre-loaded specimen
//            {"Drive", "0", "8000", "True"},
//            {"Turn", "180"},
//            {"DLift", "0"},
//            {"Lift", "1600"},
//            {"Drive", "180", "2000", "False"},
//            {"Lift", "0"},
//            {"Drive", "0", "2000", "False"},
//            {"Turn", "180"},
//            {"Drive", "180", "7500", "True"},

            //Position
            {"Drive", "90", "6000", "True"},
            {"Drive", "0", "13000", "True"},
            {"Turn", "90"},
            {"Drive", "180", "2000", "False"},
            {"Rotate", ".7"},
            {"Claw", "Close"},
            {"Claw", "Open"},
            {"Turn", "90"},
            {"DLift", ".83"},
            {"Drive", "350", "3000", "False"},

            //Grab
            {"Claw", "Close"},

            //Sleep
            {"Sleep", "500"},

            //Get in Pos
            {"Turn", "0"},
            {"Drive", "180", "4000", "True"},
            {"Turn", "-45"},
            {"Drive", "180", "4000", "False"},

            //Place
            {"Rotate", ".05"},
            {"DLift", "0.05"},

            //Sleep
            {"Sleep", "1000"},
            {"Lift", "4000"},
            {"Drive", "180", "2500", "False"},
            {"Claw", "Open"},
    };

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        turn.init(hardwareMap);
        claw.init(hardwareMap);

        waitForStart();

        for(String[] instruction : instructions){
            double theta = 0;
            double distance = 0;
            double timeout = System.nanoTime();
            switch (instruction[0]) {
                case "Drive": //PARAM: 1 - Angle, 2 - Distance, 3 - Fix Rotation (Only use when driving with 0 starting orientation)
                    double oldAngle = dt.getImu().getAngularOrientation().firstAngle;
                    theta = Double.parseDouble(instruction[1]);
                    distance = Double.parseDouble(instruction[2]);
                    dt.driveToLocation(PidConstantsDistance, theta, distance, 4);
                    //Fix Angle
                    if(instruction[3].equals("True")) {
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
                        if((System.nanoTime() - timeout)/1e9 > 3){
                            break;
                        }
                        lin.gotoPosition(distance, new CustomPID(PidConstantsAngle));
                        packet.put("Lin", lin.getPos());
                        dashboard.sendTelemetryPacket(packet);
                    }
                    lin.setPower(0);
                    break;
                case "DLift":
                    theta = Double.parseDouble(instruction[1]);
                    while(Math.abs(theta-lin.getPos()) > .05){
                        if((System.nanoTime() - timeout)/1e9 > 3){
                            break;
                        }
                        turn.gotoMaxPosition(theta);
                    }
                    turn.setPower(0);
                    break;
                case "Rotate":
                    theta = Double.parseDouble(instruction[1]);
                    claw.setRotate(theta);
                    break;
                case "Claw":
                    if (instruction[1].equals("Close")){
                        claw.setClaw(.8);
                    }else{
                        claw.setClaw(.5);
                    }
                    break;
                case "Sleep":
                    long ms = Long.parseLong(instruction[1]);
                    sleep(ms);
            }
        }
    }
}
