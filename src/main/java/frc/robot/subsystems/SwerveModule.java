// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = Math.PI;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turnMotor;

  private final CANcoder m_turningCANcoder;

  private final String moduleName;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turnPIDController = new ProfiledPIDController(1,0,0, new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, String name) {
    m_driveMotor = new TalonFX(driveMotorID);
    m_turnMotor = new TalonFX(driveMotorID);

    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_turnMotor.setNeutralMode(NeutralModeValue.Brake);

    m_turningCANcoder = new CANcoder(canCoderID);

    moduleName = name;

    m_turningCANcoder.getConfigurator().apply(new CANcoderConfiguration());

    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), new Rotation2d(getCurrentAngle()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(getCurrentAngle()));
  }

  public double getDriveDistance() {
    return m_driveMotor.getRotorPosition().getValueAsDouble() / Constants.swerveDriveGearRatio * Math.PI * Constants.swerveWheelDiameter;
  }

  public double getCurrentAngle(){
    return (m_turningCANcoder.getPosition().getValueAsDouble() * 2 * Math.PI);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getCurrentAngle()));

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(new Rotation2d(getCurrentAngle())).getCos();

    final double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
    final double turnOutput = m_turnPIDController.calculate(getCurrentAngle(), state.angle.getRadians());

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    final double turnFeedforward = m_turnFeedforward.calculate(m_turnPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turnMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public double getDriveVelocity() {
    return m_driveMotor.getRotorVelocity().getValueAsDouble() / Constants.swerveDriveGearRatio  * Math.PI * Constants.swerveWheelDiameter;
  }
}
