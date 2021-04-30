// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Drivetrain drivetrain = new Drivetrain();

  private Trajectory trajectory;

  private RamseteController ramseteController = new RamseteController();

  private Timer timer = new Timer();

  @Override
  public void robotInit() {
    String trajectoryJson = "Barrel.wpilib.json";
    /*
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory", false);
    }*/

    trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.5, 0.25),
            new Translation2d(1.0, -0.25),
            new Translation2d(1.5, 0)
        ),
        new Pose2d(0.0, 0, new Rotation2d(Math.PI)),
        new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAccel));


    drivetrain.plotTrajectory(trajectory);
  }

  @Override
  public void robotPeriodic() {
    drivetrain.periodic();
    SmartDashboard.putNumber(
        "Rotation", drivetrain.getOdometry().getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Pose Forward (X)", drivetrain.getOdometry().getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Sideways (Y)", drivetrain.getOdometry().getPoseMeters().getY());
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    drivetrain.resetOdometry(trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    if (timer.get() < trajectory.getTotalTimeSeconds()) {
      var desiredPose = trajectory.sample(timer.get());

      var refChassisSpeedsTarget = new ChassisSpeeds(desiredPose.velocityMetersPerSecond, 0.0, desiredPose.velocityMetersPerSecond * desiredPose.curvatureRadPerMeter);
      var refChassisSpeeds = ramseteController.calculate(drivetrain.getOdometry().getPoseMeters(), desiredPose);

      var targetSpeeds = Drivetrain.kinematics.toWheelSpeeds(refChassisSpeedsTarget);
      var targetSpeedsRamsete = Drivetrain.kinematics.toWheelSpeeds(refChassisSpeeds);

      SmartDashboard.putNumber("Left Current Speed", drivetrain.getWheelSpeeds().leftMetersPerSecond);
      SmartDashboard.putNumber("Left Target Speed", targetSpeeds.leftMetersPerSecond);
      SmartDashboard.putNumber("Left Ramsete", targetSpeedsRamsete.leftMetersPerSecond);

      SmartDashboard.putNumber("Right Current Speed", drivetrain.getWheelSpeeds().rightMetersPerSecond);
      SmartDashboard.putNumber("Right Target Speed", targetSpeeds.rightMetersPerSecond);
      SmartDashboard.putNumber("Right Ramsete", targetSpeedsRamsete.rightMetersPerSecond);

      drivetrain.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
    } else {
      drivetrain.drive(0.0, 0.0);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    drivetrain.drive();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}