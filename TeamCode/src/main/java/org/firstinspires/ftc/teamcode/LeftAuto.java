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

    //TODO: Tune PID to be even faster
    public static double[] PidConstantsAngle = new double[]{0.5, 140, 0.00002};
    public static double[] PidConstantsDistance = new double[]{0.00015, 0.25, 0.000001};

    //Auto-Hold things
    private boolean holdLift;
    private double holdLiftPow = 0.0;
    private boolean holdDLift;
    private double holdDLiftPos = 0.0;

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();

    public static String[][] instructions = {
            //Place pre-loaded specimen
            {"Rotate", "0.6"},
            {"Rotate", "0.5"},
            {"Drive", "180", "11000", "180"},
            {"Lift", "600"},
            {"Rotate", "0.4"},
            {"Lift", "0"},
            {"Claw", "Open"},
            {"Drive", "0", "2000", "180"},
//            {"Turn", "0"},
//            {"Drive", "180", "6000", "0"},

            //LEFT AUTO TEST
            {"Claw", "Open"}, //testing
            {"Drive", "-90", "6500", "180"},
            {"Turn", "90"},
            {"Drive", "-90", "3600", "90"},
            {"Rotate", ".5"},
            {"DLift", "1.05", "Hold"}, //Hold dlift at 1.05
            {"Sleep", "1500"},
            {"Rotate", "0.62"},
            {"Turn", "90"},
            {"Drive", "-20", "1500", "90"},

            //Grab
            {"Claw", "Close"},

            {"Sleep", "400"},

            {"DLift", "0.1", "Hold"}, //Hold dlift at 0.05

            //Sleep
            {"Sleep", "1500"},

            {"DLift", "0.1", "Release"}, //Release dlift hold

            //Get in Pos
            {"HoldLift", "-1.0"}, //Lift lift
            {"Drive", "45", "13700", "-30"}, //Drive to pos
            {"Turn", "-30"}, //Turn

            //Place
            {"Rotate", "0.05"},

            {"Sleep", "200"},

            {"Claw", "Open"},

            {"Sleep", "500"},

            {"Rotate", "0.5"},
            {"ReleaseLift"}, //Stop lifting lift

            {"HoldLift", "1.0"}, //Get lift back to zero

            {"Turn", "90"}, //Turn
            {"Drive", "180", "4000", "90"}, //Back up
            {"Drive", "-90", "11500", "90"}, //Go right
            {"DLift", "1.05", "Hold"}, //Hold dlift at 1.05
            {"Sleep", "1500"},
            {"ReleaseLift"},
            {"Rotate", "0.62"},
            {"Turn", "90"},
            {"Drive", "-20", "1500", "90"},

            //Grab #2
            {"Claw", "Close"},

            {"Sleep", "400"},

            {"DLift", "0.1", "Hold"}, //Hold at up pos

            //Sleep
            {"Sleep", "1500"},

            {"DLift", "0.1", "Release"}, //Release dlift hold

            //Get in Pos
            {"HoldLift", "-1.0"}, //Lift lift
            {"Drive", "55", "12000", "-30"}, //Drive to pos
            {"Turn", "-30"}, //Turn

            //Place
            {"Rotate", "0.05"},

            {"Sleep", "200"},

            {"Claw", "Open"},
            {"Rotate", "0.5"},

            {"Sleep", "500"},

            {"ReleaseLift"}, //Stop lifting lift

            {"Turn", "0"}, //Face Front

            {"Drive", "-90", "4000", "0"},
            {"Lift", "0"},
    };

    //Runs EVERY LOOP
    private void doLoop() {
        //Hold Lift Position
        if(holdLift){
            lin.moveLift(holdLiftPow);
        }else{
            lin.setPower(0);
        }
        if(holdDLift){
            if(Math.abs(holdDLiftPos-Math.abs(lin.getPos())) > .05){
                turn.getToPos(holdDLiftPos);
            }else{
                turn.setPower(0);
            }
        }else{
            turn.setPower(0);
        }

        //Telemetry
        packet.put("XyMoveGradient", dt.getXyGradAverage());
        packet.put("InstantGrad", dt.delF());
//        packet.put("Angle", dt.getImu().getAngularOrientation().firstAngle);
//        packet.put("X", dt.getxOdom().getCurrentPosition());
//        packet.put("Y", dt.getyOdom().getCurrentPosition());
//        packet.put("Lin", lin.getPos());
//        packet.put("Turn", turn.getPos());
//        packet.put("Speed", dt.getSpeed());
//        packet.put("Claw Rotation", claw.getRotate());
//        packet.put("Claw Open", claw.getPos());
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        turn.init(hardwareMap);
        claw.init(hardwareMap);

        claw.setClaw(.24);

        waitForStart();

        for(String[] instruction : instructions){
            boolean flag = false;
            double theta = 0;
            double distance = 0;
            double timeout = System.nanoTime();
            switch (instruction[0]) {
                case "Drive": //PARAM: 1 - Angle, 2 - Distance, 3 - Fix Rotation (Only use when driving with 0 starting orientation)
                    double oldAngle = Math.toDegrees(dt.getImu().getAngularOrientation().firstAngle);
                    theta = Double.parseDouble(instruction[1]);
                    distance = Double.parseDouble(instruction[2]);

                    //Drive
                    dt.resetOdoms();
                    while(true){
                        flag = dt.driveToLocation(PidConstantsDistance, theta, distance);
                        if((System.nanoTime()-timeout)/1e9 > 4){
                            break;
                        }
                        if(!flag){
                            break;
                        }
                        doLoop();
                    }
                    dt.zeroMotors();

                    //Fix Angle
                    timeout = System.nanoTime(); //Make sure it's basing the timeout on time passed since robot reached target loc
                    while (opModeIsActive()) {
                        flag = dt.fixAngle(PidConstantsAngle, Double.parseDouble(instruction[3]) + 180);
                        if((System.nanoTime()-timeout)/1e9 > 1){
                            break;
                        }
                        if(!flag){
                            break;
                        }
                        doLoop();
                    }
                    dt.zeroMotors();

                    break;
                case "Align":
                    theta = Double.parseDouble(instruction[1]);
                    distance = Double.parseDouble(instruction[2]); //Power
                    //Drive
                    dt.resetOdoms();
                    while(true){
                        flag = dt.driveToLocation(PidConstantsDistance, theta, distance);
                        if((System.nanoTime()-timeout)/1e9 > 4){
                            break;
                        }
                        if(!flag){
                            break;
                        }
                        doLoop();
                    }
                    dt.zeroMotors();
                    break;
                case "Turn":
                    theta = Double.parseDouble(instruction[1]);
                    dt.resetOdoms();
                    while(opModeIsActive()){
                        flag = dt.fixAngle(PidConstantsAngle, theta + 180);
                        if((System.nanoTime() - timeout)/1e9 > 3){
                            break;
                        }
                        if(!flag){
                            break;
                        }
                        doLoop();
                    }
                    dt.zeroMotors();
                    break;
                case "Lift":
                    distance = Double.parseDouble(instruction[1]);
                    //Lift to Position
                    while(Math.abs(lin.getPos() - distance) > 50) {
                        if ((System.nanoTime() - timeout) / 1e9 > 3) {
                            break;
                        }
                        if (distance - Math.abs(lin.getPos()) > 0) {
                            lin.moveLift(-1.0);
                        } else {
                            lin.moveLift(1.0);
                        }
                    }
                    lin.setPower(0);
                    break;
                case "HoldLift":
                    holdLift = true;
                    holdLiftPow = Double.parseDouble(instruction[1]);
                    break;
                case "ReleaseLift":
                    holdLift = false;
                    break;
                case "DLift":
                    theta = Double.parseDouble(instruction[1]);
                    if(instruction[2].equals("Hold")){
                        holdDLift = true;
                        holdDLiftPos = theta;
                    }else{
                        holdDLift = false;
                    }
                    break;
                case "Rotate":
                    theta = Double.parseDouble(instruction[1]);
                    claw.setRotate(theta);
                    doLoop();
                    break;
                case "Claw":
                    if (instruction[1].equals("Close")){
                        claw.setClaw(.24);
                    }else{
                        claw.setClaw(.5);
                    }
                    doLoop();
                    break;
                case "Sleep":
                    long ms = Long.parseLong(instruction[1]);
                    while((System.nanoTime() - timeout)/1e6 < ms){
                        doLoop();
                    }
                    break;
            }
        }
    }
}
