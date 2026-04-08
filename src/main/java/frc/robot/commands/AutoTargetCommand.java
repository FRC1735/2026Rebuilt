// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTargetCommand extends Command {
  /** Creates a new AutoTargetCommand. */
  private final Drive drive;

  private double xrobot = 0;
  private double yrobot = 0;
  private final double xtarget;
  private final double ytarget;
  private double distance = 0;
  private double theta = 0;
  private double speed = 0;
  private double hoodAngle = 0;

  public AutoTargetCommand(Drive drive, double xtarget, double ytarget, double speed) {
    this.drive = drive;
    this.xtarget = xtarget;
    this.ytarget = ytarget;
    this.speed = speed;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xrobot = drive.getXPosition();
    yrobot = drive.getYPosition();
    distance = SidsSmartCommand.getTargetDistance(xrobot, xtarget, yrobot, ytarget);
    theta = SidsSmartCommand.getTargetAngle(xrobot, xtarget, yrobot, ytarget);
    // rotate robot

    // spin shooter at speed

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
