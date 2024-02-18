package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_FIRST;
import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_SECOND;
import static com.example.meepmeeptesting.Intake.Height.FIVE_STACK;
import static com.example.meepmeeptesting.Intake.Height.FOUR_STACK;
import static com.example.meepmeeptesting.MainAuton.EditablePose.backdropSide;
import static com.example.meepmeeptesting.Pixel.Color.WHITE;
import static com.example.meepmeeptesting.Pixel.Color.YELLOW;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;
import static java.util.Arrays.asList;
import static java.util.Collections.swap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class MainAuton {

    static boolean isRed = true;
    static Backdrop autonBackdrop = new Backdrop();

    public static final double
            REVERSE = PI,
            LEFT = REVERSE,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

    /**
     * @return A {@link Pose2d} corresponding to the phsyical scoring location of this {@link Pixel}
     */
    public static Pose2d toPose2d(Pixel pixel) {
        return new Pose2d(
                X_BACKDROP,
                (isRed ? Y_BACKDROP_0_RED : Y_BACKDROP_0_BLUE) - ((pixel.x - 1) * MainAuton.WIDTH_PIXEL) - (pixel.y % 2 != 0 ? 0.5 * MainAuton.WIDTH_PIXEL : 0),
                PI
        );
    }

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12,
            X_SHIFT_BACKDROP_AFTER_SPIKE = 8,
            Y_SHIFT_BEFORE_SPIKE = 15,
            Y_SHIFT_AFTER_SPIKE = 26,
            Y_SHIFT_AUDIENCE_AFTER_SPIKE = 16,
            X_SHIFT_CENTER_AUDIENCE_AFTER_SPIKE = -22,
            X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE = -14,
            X_TILE = 24,
            X_INTAKING = -56,
            Y_INTAKING_1 = -12,
            Y_INTAKING_3 = -36,
            TIME_SPIKE = 0.75,
            TIME_SPIKE_TO_INTAKE_FLIP = 0.5,
            TIME_PRE_YELLOW = 0.5,
            X_SHIFT_INTAKING = 5,
            SPEED_INTAKING = 0.5,
            BOTTOM_ROW_HEIGHT = 2,
            X_BACKDROP = 52,
            Y_BACKDROP_0_BLUE = 43.9,
            Y_BACKDROP_0_RED = -28.1,
            WIDTH_PIXEL = 3.7,
            ANGLE_AWAY_TRUSS_SPIKE_APPROACH_RED = 5,
            ANGLE_AWAY_TRUSS_SPIKE_APPROACH_BLUE = 7.5;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpike = new EditablePose(15, -23, LEFT),
            nearTrussSpike = new EditablePose(5.4, -35, LEFT),
            awayTrussSpike = new EditablePose(29, -32, LEFT),
            parking = new EditablePose(X_BACKDROP, -60, LEFT),
            parked = new EditablePose(60, parking.y, LEFT),
            enteringBackstage = new EditablePose(36, -12, LEFT),
            movingToStack2 = new EditablePose(-45, -24, LEFT);

    private static Pose2d stackPos(int stack, Intake.Height height) {
        return new EditablePose(X_INTAKING + height.deltaX, stack == 3 ? Y_INTAKING_3 : stack == 2 ? movingToStack2.y : Y_INTAKING_1, LEFT).byAlliance().toPose2d();
    }

    private static void driveToStack1(TrajectorySequenceBuilder sequence, Intake.Height height) {
        sequence
                .addTemporalMarker(() -> {
//                    robot.intake.toggle();
//                    robot.intake.setHeight(height);
                })
                .setTangent(MainAuton.startPose.byAlliance().heading)
                .lineTo(MainAuton.enteringBackstage.byAlliance().toPose2d().vec())
                .setTangent(LEFT)
                .addTemporalMarker(() -> {
//                    robot.intake.toggle();
                })
                .splineTo(stackPos(1, height).vec(), LEFT)
        ;
    }

    private static void driveToStack2(TrajectorySequenceBuilder sequence, Intake.Height height) {
        Pose2d turnToStack1 = new EditablePose(X_START_LEFT + X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE, Y_INTAKING_1, LEFT).byAlliance().toPose2d();
        sequence
                .addTemporalMarker(() -> {
//                    robot.intake.setHeight(height);
                })
                .setTangent(MainAuton.startPose.byAlliance().heading)
                .splineToConstantHeading(MainAuton.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                .splineTo(turnToStack1.vec(), LEFT)
                .lineTo(movingToStack2.byAlliance().toPose2d().vec())
                .lineTo(stackPos(2, height).vec())
        ;
    }

    private static void intake2Pixels(TrajectorySequenceBuilder sequence, int stack, Intake.Height height) {
        Intake.Height height2 = height.minus(1);
        sequence
                .addTemporalMarker(() -> {
//                    robot.intake.setMotorPower(SPEED_INTAKING);
//                    while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
//                    robot.intake.setMotorPower(0);
                })
                .back(X_SHIFT_INTAKING)
                .addTemporalMarker(() -> {
//                    robot.intake.setHeight(height2);
                })
                .lineTo(stackPos(stack, height2).vec())
                .addTemporalMarker(() -> {
//                    robot.intake.setMotorPower(SPEED_INTAKING);
//                    while (robot.intake.colors[1] == Pixel.Color.EMPTY) {Thread.yield();}
//                    robot.intake.setMotorPower(0);
                })
        ;
    }

    private static void score(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, int index) {
        Pixel first = placements.get(index);
        Pixel second = placements.get(index + 1);
        sequence
                .lineTo(MainAuton.enteringBackstage.byAlliance().toPose2d().vec())

                .addTemporalMarker(() -> {
//                    robot.deposit.lift.setTargetRow(first.y);
                })
                .splineToConstantHeading(toPose2d(first).vec(), MainAuton.startPose.byAlliance().heading + REVERSE)
                .addTemporalMarker(() -> {
//                    robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(first);
                })
                .waitSeconds(TIME_DROP_FIRST)

                .addTemporalMarker(() -> {
//                    robot.deposit.lift.setTargetRow(second.y);
                })
                .lineToConstantHeading(toPose2d(second).vec())
                .addTemporalMarker(() -> {
//                    robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(second);
                })
                .waitSeconds(TIME_DROP_SECOND)
        ;
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(720);

        Pose2d startPose = MainAuton.startPose.byBoth().toPose2d();
        boolean partnerWillDoRand = false;
        boolean park = true;

        int[] ourPlacements = {1, 3, 6};
        PropDetectPipeline.Randomization rand = PropDetectPipeline.Randomization.LEFT;
        ArrayList<Pixel> placements = new ArrayList<>(asList(
                new Pixel(ourPlacements[rand.ordinal()], 0, YELLOW),
                new Pixel(1, 0, WHITE),
                new Pixel(2, 0, WHITE),
                new Pixel(1, 1, WHITE),
                new Pixel(0, 1, WHITE),
                new Pixel(1, 2, WHITE),
                new Pixel(2, 1, WHITE),
                new Pixel(2, 2, WHITE),
                new Pixel(1, 3, WHITE),
                new Pixel(0, 3, WHITE),
                new Pixel(1, 4, WHITE),
                new Pixel(0, 5, WHITE),
                new Pixel(6, 0, WHITE),
                new Pixel(5, 0, WHITE),
                new Pixel(6, 1, WHITE),
                new Pixel(5, 1, WHITE),
                new Pixel(6, 2, WHITE),
                new Pixel(6, 3, WHITE)
        ));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, toRadians(250), toRadians(250), 13.95)
//                .setConstraints(70, 90, toRadians(295), toRadians(295), 13.95)
                .setDimensions(16.42205, 17.39847)
                .followTrajectorySequence(robotdrivetrain -> {

                    if (partnerWillDoRand) {
                        autonBackdrop.add(placements.get(0));
                        placements.remove(0);
                    }
                    if (!backdropSide) swap(placements, 0, 1);

                    TrajectorySequenceBuilder sequence = robotdrivetrain.trajectorySequenceBuilder(startPose)
                            .setTangent(startPose.getHeading())
                    ;

                    if (backdropSide) {

                        boolean outer =
                                (isRed && (rand == PropDetectPipeline.Randomization.RIGHT)) ||
                                (!isRed && (rand == PropDetectPipeline.Randomization.LEFT));

                        boolean inner =
                                (isRed && (rand == PropDetectPipeline.Randomization.LEFT)) ||
                                (!isRed && (rand == PropDetectPipeline.Randomization.RIGHT));

                        if (inner) {
                            Pose2d spike = nearTrussSpike.byAlliance().flipBySide().toPose2d();
                            sequence.splineTo(spike.vec(), spike.getHeading());
                        } else {
                            sequence.lineToSplineHeading((
                                    outer ?
                                            awayTrussSpike.byAlliance().flipBySide() :
                                            centerSpike.byBoth()
                            ).toPose2d());
                        }

                        sequence
                                .addTemporalMarker(() -> {
//                                    robot.spike.toggle();
                                })
                                .waitSeconds(TIME_SPIKE)
                                .setTangent(RIGHT)
                                .UNSTABLE_addTemporalMarkerOffset(TIME_SPIKE_TO_INTAKE_FLIP, () -> {
//                                    robot.deposit.lift.setTargetRow(placements.get(0).y);
                                })
                                .splineToConstantHeading(toPose2d(placements.get(0)).vec(),
                                        outer ?
                                                isRed ? ANGLE_AWAY_TRUSS_SPIKE_APPROACH_RED : ANGLE_AWAY_TRUSS_SPIKE_APPROACH_BLUE :
                                                RIGHT
                                )
                                .waitSeconds(TIME_PRE_YELLOW)
                                .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixel();
                                    autonBackdrop.add(placements.get(0));
                                })
                                .waitSeconds(TIME_DROP_SECOND)
                        ;

                    } else {

                    }

                    if (!park) {

                        Intake.Height height = backdropSide ? FIVE_STACK : FOUR_STACK;
                        int placement = backdropSide ? 1 : 2;

                        // CYCLE 1
                        driveToStack1(sequence, height);
//                intake2Pixels(sequence, 1, height);
//                score(sequence, placements, placement);

                        // CYCLE 2
//                if (backdropSide) {
//                    driveToStack1(sequence, height.minus(2));
//                    intake2Pixels(sequence, 1, height.minus(2));
//                    score(sequence, placements, placement + 2);
//                }
                    } else if (partnerWillDoRand) {
                        sequence
                                .lineTo(parking.byAlliance().toPose2d().vec())
                                .lineTo(parked.byAlliance().toPose2d().vec())
                                ;
                    }

                    sequence.addTemporalMarker(() -> autonBackdrop.print());

                    return sequence.build();
                });

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(.85f)
                .addEntity(myBot)
                .start();
    }

    public static class EditablePose {

        public double x, y, heading;

        static boolean backdropSide = false;

        public EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public EditablePose clone() {
            return new EditablePose(x, y, heading);
        }

        public EditablePose byAlliance() {
            double alliance = isRed ? 1 : -1;
            return new EditablePose(
                    x,
                    y * alliance,
                    heading * alliance
            );
        }

        public EditablePose bySide() {
            return new EditablePose(
                    x + (backdropSide ? 0 : X_START_LEFT - X_START_RIGHT),
                    y,
                    heading
            );
        }

        public EditablePose flipBySide() {
            return new EditablePose(
                    backdropSide ? x : X_START_RIGHT + X_START_LEFT - x,
                    y,
                    backdropSide ? heading : Math.PI - heading
            );
        }

        public EditablePose byBoth() {
            return byAlliance().bySide();
        }

        public Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}