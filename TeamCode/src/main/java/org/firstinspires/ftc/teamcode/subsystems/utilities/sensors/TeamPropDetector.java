package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
public class TeamPropDetector {

    private PropDetectPipeline.Randomization location = PropDetectPipeline.Randomization.CENTER;

    private final OpenCvCamera camera;

    private final PropDetectPipeline pipeline;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     * @param cameraRotation  physical orientation of camera
     */
    public TeamPropDetector(HardwareMap hardwareMap, OpenCvCameraRotation cameraRotation, String cameraName, Telemetry t) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, cameraName),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        pipeline = new PropDetectPipeline(t);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, cameraRotation);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void run () {
        location = pipeline.getLocation();
    }
    /**
     * Closes the camera
     */
    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
