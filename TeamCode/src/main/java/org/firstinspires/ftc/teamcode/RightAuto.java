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

@Autonomous(name="RightAuto", group="Linear OpMode")
@Config
public class RightAuto extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double[] PidConstantsAngle = new double[]{1, 350, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.15, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();

    public static String[][] instructions = {
            //Place pre-loaded specimen
            {"Claw", "Close"},
            {"Rotate", "0.5"},
            {"Drive", "0", "9000", "True"},
            {"Turn", "180"},
            {"DLift", "0"},
            {"Lift", "600"},
            {"Rotate", "0.4"},
            {"Drive", "180", "2000", "False"},
            {"Lift", "0"},
            {"Claw", "Open"},
            {"Drive", "0", "2000", "False"},
            {"Turn", "0"},
            {"Drive", "180", "7500", "True"},

            //Test Right Auton (Figure NNN - "Pushbot")
            {"Drive", "270", "10000", "True"},
            {"Drive", "0", "17000", "True"},

            {"Drive", "270", "2000", "True"},
            {"Drive", "180", "14000", "True"},
            {"Drive", "0", "14000", "True"},

            {"Drive", "270", "2000", "True"},
            {"Drive", "180", "14000", "True"},
            {"Drive", "0", "14000", "True"},

            {"Drive", "270", "1000", "True"},
            {"Drive", "180", "15000", "True"},
    };

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        turn.init(hardwareMap);
        claw.init(hardwareMap);

        waitForStart();

        claw.setClaw(.24);

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
                    while(Math.abs(lin.getPos() - distance) > 50){
                        if((System.nanoTime() - timeout)/1e9 > 3){
                            break;
                        }
                        if(distance - Math.abs(lin.getPos()) > 0){
                            lin.moveLift(-1.0);
                        }else{
                            lin.moveLift(1.0);
                        }
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
                    dt.zeroMotors();
                    break;
                case "Rotate":
                    theta = Double.parseDouble(instruction[1]);
                    claw.setRotate(theta);
                    break;
                case "Claw":
                    if (instruction[1].equals("Close")){
                        claw.setClaw(.24);
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

