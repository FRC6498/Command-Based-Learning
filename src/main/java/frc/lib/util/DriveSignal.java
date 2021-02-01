// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

/**
 * A drivetrain command consisting of lift and right power levels and wther we are braking or not.
*/
public class DriveSignal {
    protected double leftSidePower, rightSidePower;
    protected boolean brakeOn;
    private String brakeActive = ", BRAKE";
    private String brakeString = "";

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean braking) {
        leftSidePower = left;
        rightSidePower = right;
        brakeOn = braking;
        if (braking) {
            brakeString = brakeActive;
        }
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0,0);
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return leftSidePower;
    }

    public double getRight() {
        return rightSidePower;
    }

    public boolean getBrakeMode() {
        return brakeOn;
    }

    @Override
    public String toString() {
        return String.format("L: %f, R: %f%S", leftSidePower, rightSidePower, brakeString);
    }
}
