// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  
  Translation2d m_frontLeftLocation = new Translation2d(0.3048, 0.3048);
  Translation2d m_frontRightLocation = new Translation2d(0.3048, -0.3048);
  Translation2d m_backLeftLocation = new Translation2d(-0.3048, 0.3048);
  Translation2d m_backRightLocation = new Translation2d(-0.3048, -0.3048);

  private final SwerveModule m_frontLeft = new SwerveModule(41, 42, 43, "front right");
  private final SwerveModule m_frontRight = new SwerveModule(31, 32, 33, "front left");
  private final SwerveModule m_backLeft = new SwerveModule(12, 11, 13, "back right");
  private final SwerveModule m_backRight = new SwerveModule(22, 21, 23, "back left");

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
  
  public DriveSubsystem() {
    m_gyro.calibrate();
  }

  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void updateOdometry() {
    m_odometry.update(
        getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(-m_gyro.getGyroAngleZ() / 57.295779513);
  }
}
