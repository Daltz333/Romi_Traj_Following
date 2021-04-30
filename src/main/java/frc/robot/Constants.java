// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
  public static final double kCountsPerRevolution = 1440.0;
  public static final double kWheelDiameterMeters = 0.07;

  public static final int kLeftMotorPort = 0;
  public static final int kRightMotorPort = 1;

  public static final int kLeftEncoderA = 4;
  public static final int kLeftEncoderB = 5;

  public static final int kRightEncoderA = 6;
  public static final int kRightEncoderB = 7;

  // constants from characterization
  public static final double kS = 0.629;
  public static final double kV = 6.83;
  public static final double kA = 0.0389;
  public static final double kTrackWidth = 0.142072613;

  public static final double kPLeft = 0.42;
  public static final double kPRight = 0.285;

  public static final double kMaxSpeed = 0.8; // 1.6m/s
  public static final double kMaxAccel = 0.6;
}
