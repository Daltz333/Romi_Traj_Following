// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class Drivetrain {
  // create our drivetrain components
  private final Spark leftMotor = new Spark(Constants.kLeftMotorPort);
  private final Spark rightMotor = new Spark(Constants.kRightMotorPort);

  private final Encoder leftEncoder = 
      new Encoder(Constants.kLeftEncoderA, Constants.kLeftEncoderB);
  private final Encoder rightEncoder =
      new Encoder(Constants.kRightEncoderA, Constants.kRightEncoderB);

  private final RomiGyro gyro = new RomiGyro();

  // teleop drive
  private final DifferentialDrive diffyDrive = new DifferentialDrive(leftMotor, rightMotor);
  private final XboxController controller = new XboxController(0);

  // path following instantiations
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(gyro.getRotation2d());

  public static final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.kTrackWidth);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(Constants.kS, Constants.kV);

  private final PIDController leftPid = new PIDController(Constants.kPLeft, 0, 0);
  private final PIDController rightPid = new PIDController(Constants.kPRight, 0, 0);

  private final Field2d field = new Field2d();

  /** Creates a new RomiDrivetrain. */
  public Drivetrain() {
    leftEncoder.setDistancePerPulse(
        (Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    rightEncoder.setDistancePerPulse(
        (Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);

    resetOdometry();
    SmartDashboard.putData(field);
  }

  public void drive() {
    var forward = controller.getY(Hand.kLeft);
    var turn = controller.getX(Hand.kRight);

    diffyDrive.curvatureDrive(forward, turn, controller.getBumper(Hand.kRight));
  }

  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void plotTrajectory(Trajectory trajectory) {
    ArrayList<Pose2d> poses = new ArrayList<>();

    for (Trajectory.State pose : trajectory.getStates()) {
      poses.add(pose.poseMeters);
    }

    field.getObject("foo").setPoses(poses);
  }

  public void resetOdometry() {
    leftEncoder.reset();
    rightEncoder.reset();
    gyro.reset();
  }

  public void resetOdometry(Pose2d initialPose) {
    leftEncoder.reset();
    rightEncoder.reset();
    gyro.reset();

    odometry.resetPosition(initialPose, gyro.getRotation2d());
  }

  // update our odometry periodically
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());

    field.getObject("Robot").setPose(odometry.getPoseMeters());
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    var leftOutput = leftPid.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    var rightOutput = rightPid.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);

    leftMotor.setVoltage(leftOutput + leftFeedforward);
    rightMotor.setVoltage(-(rightOutput + rightFeedforward)); //negate right side

    diffyDrive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  // return odometry object
  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  // return left meters
  public double getLeftDistanceMeters() {
    return leftEncoder.getDistance();
  }

  // return right meters
  public double getRightDistanceMeters() {
    return rightEncoder.getDistance();
  }
}
