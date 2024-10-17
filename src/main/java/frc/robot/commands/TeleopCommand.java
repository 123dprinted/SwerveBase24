// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController driveController;

  private final SlewRateLimiter m_xspeedLimiter;
  private final SlewRateLimiter m_yspeedLimiter;
  private final SlewRateLimiter m_rotLimiter;

  boolean fieldRelative;

  public TeleopCommand(DriveSubsystem subsystem, CommandXboxController controller) {
    driveSubsystem = subsystem;
    driveController = controller;

    m_xspeedLimiter = new SlewRateLimiter(3);
    m_yspeedLimiter = new SlewRateLimiter(3);
    m_rotLimiter = new SlewRateLimiter(3);

    fieldRelative = true;
    
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driveController.getLeftY(), 0.02))
            * DriveSubsystem.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driveController.getLeftX(), 0.02))
            * DriveSubsystem.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(driveController.getRightX(), 0.02))
            * DriveSubsystem.kMaxAngularSpeed;

    driveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, fieldRelative);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
