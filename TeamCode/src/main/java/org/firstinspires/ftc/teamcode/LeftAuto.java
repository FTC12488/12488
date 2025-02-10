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

import java.security.PrivateKey;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name="LeftAuto", group="Linear OpMode")
@Config
public class LeftAuto extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    //TODO: Tune PID to be even faster
    public static double[] PidConstantsAngle = new double[]{0.5, 140, 0.00002};
    public static double[] PidConstantsDistance = new double[]{0.00015, 0.25, 0.000001};

    //Time Tracker
    private double timer;

    //Auto targets
    private HashMap<String, String> targets = new HashMap<String, String>(Map.of(
            "Drive", "0.0",
            "DAngle", "0.0",
            "Turn", "0.0",
            "Lift", "0.0",
            "DLift", "0.0",
            "Claw", "Open",
            "Rotate", "0.5"
    ));

    /// Auto Instructions
    /// Key is the timestamp in tenths of a second (IE: 45 = 4.5 seconds since start of auto)
    /// Value is a list containing the actual instruction.
        /// Value[0] is the name
        /// Value[1] is the value
        /// Value[1-3] (drive only): Distance, Drive Angle, Final Orientation
    private HashMap<Integer, String[]> instructions = new HashMap<Integer, String[]>(Map.of(
            0, new String[]{"Drive", "0.0", "0.0", "0.0"},
            3, new String[]{"Lift", "0.0"},
            4, new String[]{"DLift", "0.0"},
            5, new String[]{"Claw", "Open"},
            6, new String[]{"Rotate", "0.5"}
    ));

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();


    //Runs EVERY LOOP
    private void doLoop() {
        try{
            //Drive train
            dt.moveRobot(PidConstantsDistance, PidConstantsAngle, Double.parseDouble(targets.get("Drive")), Double.parseDouble(targets.get("DAngle")), Double.parseDouble(targets.get("Turn")));
            //Linear Lift
            lin.moveLift(Double.parseDouble(targets.get("Lift")));
            //"Down" Lift
            turn.getToPos(Double.parseDouble(targets.get("Dlift")));
            //Rotate Claw
            claw.setRotate(Double.parseDouble(targets.get("Rotate")));
            //Claw
            if (targets.get("Claw").equals("Open")){
                claw.setClaw(5);
            }else{
                claw.setClaw(.24);
            }
        }catch (NullPointerException exception){
            packet.put("Null Pointer. One or more values are null:", exception);
        }

        //Telemetry
        packet.put("InstantGrad", dt.delF());
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

        timer = System.nanoTime();
        while(opModeIsActive()){
            int timestamp = (int) Math.round((System.nanoTime() - timer) / 1e8);

            String[] instruction = instructions.get(timestamp);
            if (instruction != null){
                if(instruction[0].equals("Drive")){
                    targets.put("Drive", instruction[1]);
                    targets.put("DAngle", instruction[2]);
                    targets.put("Turn", instruction[3]);
                }else{
                    targets.put(instruction[0], instruction[1]);
                }
            }

            doLoop();
        }
    }
}
