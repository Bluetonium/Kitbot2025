// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

// Class to drive the robot over CAN
public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  private final DifferentialDrive drive;
  private final Pigeon2 robotGyro;
  private Pose2d robotPose;
  private final DifferentialDriveOdometry robotOdometry;
  private final RelativeEncoder m_rightEncoder;
  private final RelativeEncoder m_leftEncoder;

  public CANDriveSubsystem() {

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      robotConfig = new RobotConfig(40, 7, null, Constants.WHEELBASE);
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential
                                   // drive trains
        robotConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // create brushed motors for drive
    robotGyro = new Pigeon2(Constants.GRYO_ID);
    leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);
    m_rightEncoder = rightLeader.getEncoder();
    m_leftEncoder = leftLeader.getEncoder();
    robotOdometry = new DifferentialDriveOdometry(robotGyro.getRotation2d(),
        m_rightEncoder.getPosition(), m_leftEncoder.getPosition());

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set conifg to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(false);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    var gyroAngle = robotGyro.getRotation2d();
    robotPose = robotOdometry.update(gyroAngle,
        getRobotDistance(Constants.GEARRATIO, m_leftEncoder.getPosition(), Constants.WHEELDIAMETER),
        getRobotDistance(Constants.GEARRATIO, m_leftEncoder.getPosition(), Constants.WHEELDIAMETER));
  }

  // /8.461
  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);

  }

  public double getRobotDistance(double gearRatio, double motorRotations, double wheelDiameter) {
    // Calculate the distance traveled by the wheel
    double wheelCircumference = Math.PI * wheelDiameter;
    double distanceTraveled = motorRotations * gearRatio * wheelCircumference;
    return distanceTraveled;
    // kill me
  }

  public Pose2d getPose() {
    return robotPose;
  }

  public void resetPose(Pose2d newpos) {
    robotOdometry.resetPose(newpos);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Get the wheel velocities (in meters per second).
    double leftVelocity = getRobotDistance(Constants.GEARRATIO, m_leftEncoder.getVelocity(), Constants.WHEELDIAMETER);
    double rightVelocity = getRobotDistance(Constants.GEARRATIO, m_rightEncoder.getVelocity(), Constants.WHEELDIAMETER);

    // Calculate the linear velocity (vx) and angular velocity (omega).
    double vx = (leftVelocity + rightVelocity) / 2.0; // average of left and right velocities
    double omega = (rightVelocity - leftVelocity) / Constants.WHEELBASE; // difference between wheel velocities divided
                                                                         // by the wheelbase

    // Return the chassis speeds (vx, vy, omega). vy is 0 for differential drive.
    return new ChassisSpeeds(vx, 0.0, omega);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {

    double vx = chassisSpeeds.vxMetersPerSecond;
    double omega = chassisSpeeds.omegaRadiansPerSecond;

    double leftSpeed = vx - (omega * Constants.WHEELBASE / 2.0);
    double rightSpeed = vx + (omega * Constants.WHEELBASE / 2.0);

    double leftMotorSpeed = leftSpeed;
    double rightMotorSpeed = rightSpeed;

    leftLeader.set(leftMotorSpeed);
    rightLeader.set(rightMotorSpeed);
  }

}
