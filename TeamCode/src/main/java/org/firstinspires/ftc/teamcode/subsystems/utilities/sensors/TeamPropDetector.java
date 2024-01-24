package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.LEFT;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline;

@Config
public class TeamPropDetector {

    public static double MAX_DISTANCE = 50;

    private PropDetectPipeline.Randomization location = CENTER;

    private final DistanceSensor leftSensor, rightSensor;

    private double leftDistance, rightDistance;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     */
    public TeamPropDetector(HardwareMap hardwareMap) {
        leftSensor = hardwareMap.get(DistanceSensor.class, "left distance");
        rightSensor = hardwareMap.get(DistanceSensor.class, "right distance");
    }

    public PropDetectPipeline.Randomization run() {
        return location = (
                (leftDistance = leftSensor.getDistance(INCH)) <= MAX_DISTANCE ? LEFT :
                (rightDistance = rightSensor.getDistance(INCH)) <= MAX_DISTANCE ? RIGHT :
                CENTER
        );
    }

    public PropDetectPipeline.Randomization getLocation() {
        return location;
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("Left distance", leftDistance);
        mTelemetry.addData("Right distance", rightDistance);
    }
}