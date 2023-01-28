// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double controllerDeadzone = 0.125;
  }

  public static final int falconEncoderResolution = 2048;

  //TODO: Change to real values
  public static final double swerveWheelRadius = 3.95;
  public static final double swerveDriveGearRatio = 6.75;
  public static final double swerveTurnGearRatio = 150.0 / 7.0;
  public static final double moduleMaxAngularVelocity = Math.PI;
  public static final double moduleMaxAngularAcceleration = 2 * Math.PI;
}
