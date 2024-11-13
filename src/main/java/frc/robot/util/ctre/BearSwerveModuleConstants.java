package frc.robot.util.ctre;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;

public class BearSwerveModuleConstants {
    public static long createNativeInstance(SwerveModuleConstants[] constantsArr) {
        final long retval = SwerveJNI.JNI_CreateModuleConstantsArr(constantsArr.length);
        for (int i = 0; i < constantsArr.length; ++i) {
            final var constants = constantsArr[i];
            SwerveJNI.JNI_SetModuleConstants(
                retval, i,
                constants.SteerMotorId,
                constants.DriveMotorId,
                constants.CANcoderId,
                constants.CANcoderOffset,
                constants.LocationX,
                constants.LocationY,
                constants.DriveMotorInverted,
                constants.SteerMotorInverted,
                constants.DriveMotorGearRatio,
                constants.SteerMotorGearRatio,
                constants.CouplingGearRatio,
                constants.WheelRadius,
                constants.SteerMotorClosedLoopOutput.value,
                constants.DriveMotorClosedLoopOutput.value,
                constants.SlipCurrent,
                constants.SpeedAt12Volts,
                constants.FeedbackSource.value,
                constants.SteerInertia,
                constants.DriveInertia,
                constants.SteerFrictionVoltage,
                constants.DriveFrictionVoltage
            );
        }
        return retval;
    }
}
