package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.GREEN;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_0_PIXELS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_1_PIXEL;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIVOTING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXELS_FALLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXELS_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXEL_1_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.WAITING_FOR_DEPOSIT;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.ColorSensor;

@Config
public final class Intake {

    public static double
            ANGLE_PIVOT_OFFSET = 9,
            ANGLE_PIVOT_FLOOR_CLEARANCE = 2.5,
            ANGLE_PIVOT_TRANSFERRING = 196.5,
            ANGLE_PIVOT_VERTICAL = 110,
            ANGLE_LATCH_INTAKING = 105,
            ANGLE_LATCH_LOCKED = 159,
            ANGLE_LATCH_TRANSFERRING = 0,
            ANGLE_STACK_2 = 8,
            ANGLE_STACK_3 = 10.5,
            ANGLE_STACK_4 = 15.5,
            ANGLE_STACK_5 = 19.61,
            TIME_PIXEL_1_SETTLING = 0.35,
            TIME_PIVOTING = 0,
            TIME_SETTLING = 0.2,
            TIME_INTAKE_FLIP_TO_LIFT = 0.2,
            TIME_REVERSING = 0.175,
            SPEED_SLOW_REVERSING = -0.2,
            SPEED_REVERSING = -0.6,
            COLOR_SENSOR_GAIN = 1;

    /**
     * HSV value bound for intake pixel detection
     */
    public static HSV
            minWhite = new HSV(
                    0,
                    0,
                    0.03
            ),
            maxWhite = new HSV(
                    360,
                    0.6,
                    0.45
            ),
            minPurple = new HSV(
                    205,
                    0.55,
                    0.01
            ),
            maxPurple = new HSV(
                    225,
                    1,
                    0.35
            ),
            minYellow = new HSV(
                    90,
                    0.4,
                    0.01
            ),
            maxYellow = new HSV(
                    125,
                    1,
                    0.15
            ),
            minGreen = new HSV(
                    130,
                    0.5,
                    0.01
            ),
            maxGreen = new HSV(
                    160,
                    1,
                    0.2
            );

    private final MotorEx motor;

    private final ColorSensor[] sensors;
    private final HSV[] HSVs = {new HSV(), new HSV()};
    private final Pixel.Color[] reads = {EMPTY, EMPTY};
    public final Pixel.Color[] colors = {EMPTY, EMPTY};

    private final TouchSensor pivotSensor;

    private final SimpleServoPivot pivot, latch;

    private Intake.State state = HAS_0_PIXELS;
    private Intake.Height height = FLOOR;

    private final ElapsedTime timer = new ElapsedTime(), timeSinceRetracted = new ElapsedTime();

    private boolean pixelsTransferred = false, isIntaking = false;
    private int intakingAmount = 2;
    private double motorPower;

    enum State {
        HAS_0_PIXELS,
        PIXEL_1_SETTLING,
        HAS_1_PIXEL,
        WAITING_FOR_DEPOSIT,
        PIVOTING,
        PIXELS_FALLING,
        PIXELS_SETTLING,
        RETRACTED,
    }

    public enum Height {
        FLOOR,
        TWO_STACK,
        THREE_STACK,
        FOUR_STACK,
        FIVE_STACK;

        private static final Intake.Height[] values = values();

        public Intake.Height minus(int less) {
            return values[max(ordinal() - less, 0)];
        }

        public double getAngle() {
            switch (this) {
                default: case FLOOR: return 0;
                case TWO_STACK: return ANGLE_STACK_2;
                case THREE_STACK: return ANGLE_STACK_3;
                case FOUR_STACK: return ANGLE_STACK_4;
                case FIVE_STACK: return ANGLE_STACK_5;
            }
        }
    }


    Intake(HardwareMap hardwareMap) {

        pivot = new SimpleServoPivot(
                ANGLE_PIVOT_OFFSET,
                ANGLE_PIVOT_OFFSET + ANGLE_PIVOT_TRANSFERRING,
                getAxonServo(hardwareMap, "intake right"),
                getReversedServo(getAxonServo(hardwareMap, "intake left"))
        );
        pivot.setActivated(true);

        latch = new SimpleServoPivot(
                ANGLE_LATCH_TRANSFERRING,
                ANGLE_LATCH_LOCKED,
                getGoBildaServo(hardwareMap, "latch right"),
                getReversedServo(getGoBildaServo(hardwareMap, "latch left"))
        );

        motor = new MotorEx(hardwareMap, "intake", RPM_1620);
        motor.setZeroPowerBehavior(FLOAT);
        motor.setInverted(true);

        sensors = new ColorSensor[]{
            new ColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN),
            new ColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN),
        };

        pivotSensor = hardwareMap.get(TouchSensor.class, "intake pivot sensor");

        timer.reset();
    }

    /**
     * @return The {@link Pixel.Color} corresponding to the provided {@link HSV} as per the tuned value bounds
     */
    public static Pixel.Color fromHSV(HSV hsv) {
        return
                hsv.between(minPurple, maxPurple) ? PURPLE :
                hsv.between(minGreen, maxGreen) ? GREEN :
                hsv.between(minYellow, maxYellow) ? YELLOW :
                hsv.between(minWhite, maxWhite) ? WHITE :
                EMPTY;
    }

    void run(int pixelsInDeposit, boolean depositIsExtended, boolean liftIsScoring) {

        if (pixelsTransferred) pixelsTransferred = false;

        boolean depositIsRunning = liftIsScoring || depositIsExtended;

        switch (state) {
            case HAS_0_PIXELS:

                boolean bottomFull = (colors[0] = readColorSensor(0)) != EMPTY;
                if (bottomFull || !isIntaking) {
                    state = PIXEL_1_SETTLING;
                    timer.reset();
                } else break;

            case PIXEL_1_SETTLING:

                if (!isIntaking || timer.seconds() >= TIME_PIXEL_1_SETTLING) state = HAS_1_PIXEL;
                else break;

            case HAS_1_PIXEL:

                boolean topFull = (colors[1] = readColorSensor(1)) != EMPTY;
                if (topFull || !isIntaking || intakingAmount == 1) {
                    if (colors[0] != EMPTY) latch.setActivated(true);
                    state = WAITING_FOR_DEPOSIT;
                } else break;

            case WAITING_FOR_DEPOSIT:

                boolean bottomEmpty = colors[0] == EMPTY;
                int pixelsInIntake = (colors[1] == EMPTY ? 0 : 1) + (bottomEmpty ? 0 : 1);

                if (bottomEmpty) {
                    retract();
                    break;
                } else if (!depositIsExtended && pixelsInIntake + pixelsInDeposit <= 2) {
                    state = PIVOTING;
                    pivot.setActivated(true);
                    timer.reset();
                    setHeight(FLOOR);
                } else break;

            case PIVOTING:

                if (pivotSensor.isPressed() && timer.seconds() >= TIME_PIVOTING) {
                    state = PIXELS_FALLING;
                    latch.setActivated(false);
                } else {
                    setMotorPower(timer.seconds() <= TIME_REVERSING ? SPEED_REVERSING : SPEED_SLOW_REVERSING);
                    break;
                }

            case PIXELS_FALLING:

                if (readColorSensor(1) == EMPTY && readColorSensor(0) == EMPTY) {
                    state = PIXELS_SETTLING;
                    timer.reset();
                } else break;

            case PIXELS_SETTLING:

                pixelsTransferred = timer.seconds() >= TIME_SETTLING;
                if (pixelsTransferred) retract();
                else break;

            case RETRACTED:

                if (isIntaking) {
                    state = HAS_0_PIXELS;
                    pivot.setActivated(false);
                } else {
                    pivot.setActivated(!depositIsRunning);
                    break;
                }

        }


        if (pivot.isActivated()) timeSinceRetracted.reset();

        boolean doneIntaking = state.ordinal() >= WAITING_FOR_DEPOSIT.ordinal();
        boolean donePivoting = state.ordinal() > PIVOTING.ordinal();

        if (donePivoting || state == PIXEL_1_SETTLING) setMotorPower(0);

        double ANGLE_PIVOT_DOWN =
                doneIntaking ? ANGLE_PIVOT_VERTICAL :
                height != FLOOR ? height.getAngle() :
                motorPower > 0 ? 0 :
                ANGLE_PIVOT_FLOOR_CLEARANCE;

        pivot.updateAngles(
                ANGLE_PIVOT_OFFSET + ANGLE_PIVOT_DOWN,
                ANGLE_PIVOT_OFFSET + ANGLE_PIVOT_TRANSFERRING
        );

        double ANGLE_LATCH_UNLOCKED = doneIntaking ? ANGLE_LATCH_TRANSFERRING : ANGLE_LATCH_INTAKING;

        latch.updateAngles(ANGLE_LATCH_UNLOCKED, ANGLE_LATCH_LOCKED);

        pivot.run();
        latch.run();

        motor.set(motorPower);
    }

    private Pixel.Color readColorSensor(int i) {
        sensors[i].update();
        return reads[i] = fromHSV(HSVs[i] = sensors[i].getHSV());
    }

    private void retract() {
        state = RETRACTED;
        isIntaking = false;
        setIntakingAmount(2);
    }

    boolean clearOfDeposit() {
        return timeSinceRetracted.seconds() >= TIME_INTAKE_FLIP_TO_LIFT;
    }

    boolean pixelsTransferred() {
        return pixelsTransferred;
    }

    public void setHeight(Intake.Height height) {
        this.height = height;
    }

    public void setMotorPower(double motorPower) {
        if (motorPower != 0) isIntaking = true;
        this.motorPower = motorPower;
    }

    public void setIntakingAmount(int pixelCount) {
        this.intakingAmount = clip(pixelCount, 1, 2);
    }

    public void setExtended(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    public void toggle() {
        setExtended(!isIntaking);
    }

    void printTelemetry() {
        mTelemetry.addLine("Intaking " + intakingAmount + " pixels");
        mTelemetry.addLine();
        mTelemetry.addData("Top color", reads[1].name());
        mTelemetry.addData("Bottom color", reads[0].name());
    }

    void printNumericalTelemetry() {
        HSVs[1].toTelemetry("Top HSV");
        mTelemetry.addLine();
        HSVs[0].toTelemetry("Bottom HSV");
    } 
}
