package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonVars.X_START_LEFT;
import static com.example.meepmeeptesting.AutonVars.X_START_RIGHT;
import static com.example.meepmeeptesting.AutonVars.isRed;

import static java.lang.Math.toDegrees;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class EditablePose {

    public double x, y, heading;

    public EditablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public String toString() {
        return x + ", " + y + ", " + toDegrees(heading);
    }

    public EditablePose(Pose2d pose) {
        this(pose.getX(), pose.getY(), pose.getHeading());
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
                x + (AutonVars.backdropSide ? 0 : X_START_LEFT - X_START_RIGHT),
                y,
                heading
        );
    }

    public EditablePose flipBySide() {
        return new EditablePose(
                AutonVars.backdropSide ? x : X_START_RIGHT + X_START_LEFT - x,
                y,
                AutonVars.backdropSide ? heading : Math.PI - heading
        );
    }

    public EditablePose byBoth() {
        return byAlliance().bySide();
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, heading);
    }
}
