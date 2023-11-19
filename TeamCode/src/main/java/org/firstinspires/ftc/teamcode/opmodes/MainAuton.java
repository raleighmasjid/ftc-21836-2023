package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Config
@Autonomous(group = "21836 Autonomous", preselectTeleOp = "MainTeleOp")
public class MainAuton extends LinearOpMode {

    // Declare objects:
    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;
    GamepadEx Gamepad1;
    public static boolean red = true;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        myTelemetry = new MultipleTelemetry(telemetry);

        // Initialize internal hub representations:
        // Switch hubs to manually reset sensor inputs when we tell it to:
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Initialize gamepad (ONLY FOR INIT, DON'T CALL DURING WHILE LOOP)
        Gamepad1 = new GamepadEx(gamepad1);

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        boolean right = true;
        while (opModeInInit() && !(Gamepad1.isDown(RIGHT_BUMPER) && Gamepad1.isDown(LEFT_BUMPER))) {
            Gamepad1.readButtons();
            if (Gamepad1.wasJustPressed(DPAD_RIGHT)) right = true;
            if (Gamepad1.wasJustPressed(DPAD_LEFT)) right = false;
            if (Gamepad1.wasJustPressed(B)) red = true;
            if (Gamepad1.wasJustPressed(X)) red = false;
            myTelemetry.addLine("Selected " + (red ? "RED" : "BLUE") + " " + (right ? "RIGHT" : "LEFT"));
            myTelemetry.addLine("Press both shoulder buttons to confirm!");
            myTelemetry.update();
        }
        myTelemetry.addLine("Confirmed " + (red ? "RED" : "BLUE") + " " + (right ? "RIGHT" : "LEFT"));
        myTelemetry.update();

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            for (LynxModule hub : hubs) hub.clearBulkCache();

            // Push telemetry data
            myTelemetry.update();
        }
    }
}