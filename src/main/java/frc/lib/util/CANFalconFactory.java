// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Creates CTRE Controller objects and configures all the parameters we care about.
 * Closed-loop and sensor parameters are not set, as these
 * are expected to be set by the application.
 */
public class CANFalconFactory {

    public static WPI_TalonFX createFalcon(int deviceID, boolean inverted, NeutralMode neutralMode,
    FeedbackDevice device, int controlID, boolean reverseSensor) {
        WPI_TalonFX falcon = new WPI_TalonFX(deviceID);

        falcon.clearStickyFaults();
        falcon.setInverted(inverted);
        falcon.setNeutralMode(neutralMode);
        falcon.configSelectedFeedbackSensor(device);
        falcon.setSensorPhase(reverseSensor);
        return falcon;
    }

    public static WPI_TalonFX tunePID(WPI_TalonFX falcon, int id, double P, double I, double D, double F) {
        falcon.config_kP(id, P);
        falcon.config_kI(id, I);
        falcon.config_kD(id, D);
        falcon.config_kF(id, F);
        return falcon;
    }

    public static WPI_TalonFX setupHardLimits(WPI_TalonFX falcon, LimitSwitchSource forwardSource, 
    LimitSwitchNormal forwardNormal, boolean clearPosOnForward, LimitSwitchSource reverseSource, 
    LimitSwitchNormal reverseNormal, boolean clearPosOnReverse){

        falcon.configForwardLimitSwitchSource(forwardSource, forwardNormal);
        falcon.configReverseLimitSwitchSource(reverseSource, reverseNormal);

        falcon.configClearPositionOnLimitF(clearPosOnForward, 0);
        falcon.configClearPositionOnLimitR(clearPosOnReverse, 0);

       return falcon; 
    }

    public static WPI_TalonFX setupSoftLimits(WPI_TalonFX falcon, boolean forwardEnable, int forwardThreshold,
    boolean reverseEnable, int reverseThreshold){

        falcon.configForwardSoftLimitThreshold(forwardThreshold);
        falcon.configReverseSoftLimitThreshold(reverseThreshold);

        falcon.configForwardSoftLimitEnable(forwardEnable);
        falcon.configReverseSoftLimitEnable(reverseEnable);

        return falcon;
    }
}
