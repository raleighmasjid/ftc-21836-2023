package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Servo(s) with two set positions <p>
 * Controlled by {@link #toggle} and {@link #setActivated}
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class SimpleServoPivot {

    public static SimpleServo getReversedServo(SimpleServo servo) {
        servo.setInverted(true);
        return servo;
    }

    public static SimpleServo getAxonServo(HardwareMap hardwareMap, String name) {
        return new SimpleServo(hardwareMap, name, 0, 355);
    }

    public static SimpleServo getGoBildaServo(HardwareMap hardwareMap, String name) {
        return new SimpleServo(hardwareMap, name, 0, 300);
    }

    private final SimpleServo[] servos;

    private double ANGLE_A, ANGLE_B;

    private boolean activated = false;

    public SimpleServoPivot(double ANGLE_A, double ANGLE_B, SimpleServo... servos) {
        this.servos = servos;
        updateAngles(ANGLE_A, ANGLE_B);
    }

    public void updateAngles(double ANGLE_A, double ANGLE_B) {
        this.ANGLE_A = ANGLE_A;
        this.ANGLE_B = ANGLE_B;
    }

    /**
     * Toggles the state of the {@link #servos}
     */
    public void toggle() {
        setActivated(!activated);
    }

    /**
     * Set state of the {@link #servos}
     *
     * @param activated False for position A, true for position B
     */
    public void setActivated(boolean activated) {
        this.activated = activated;
    }

    /**
     * Get state of the {@link #servos} <p>
     * False if position A (default) <p>
     * True if in position B
     */
    public boolean getActivated() {
        return activated;
    }

    /**
     * Hold {@link #servos} position
     */
    public void run() {
        for (SimpleServo servo : servos) servo.turnToAngle(activated ? ANGLE_B : ANGLE_A);
    }
}
