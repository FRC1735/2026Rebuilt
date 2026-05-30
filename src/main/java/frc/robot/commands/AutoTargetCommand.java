// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooterhood.ShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTargetCommand extends Command {
  /** Creates a new AutoTargetCommand. */
  private final Drive drive;

  private final ShooterHood hood;

  private double xrobot = 0;
  private double yrobot = 0;
  private final double xtarget;
  private final double ytarget;
  private double distance = 0;
  private double theta = 0;
  private double speed = 0;
  private double hoodAngle = 0;
  private double encoder = 0;

  public AutoTargetCommand(
      Drive drive, ShooterHood hood, boolean alliance, Alliance redblue, double speed) {

    this.drive = drive;
    if (redblue == Alliance.Red && alliance) {
      this.xtarget = 11.915;
    } else if (redblue == Alliance.Blue && alliance) {
      this.xtarget = 4.626;
    } else {
      this.xtarget = 0;
    }
    this.ytarget = 4.034;
    this.speed = speed;
    this.encoder = encoder;
    this.hood = hood;
    addRequirements(this.drive, this.hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xrobot = drive.getXPosition();
    yrobot = drive.getYPosition();
    distance = SidsSmartCommand.getTargetDistance(xrobot, xtarget, yrobot, ytarget);
    theta = SidsSmartCommand.getTargetAngle(xrobot, xtarget, yrobot, ytarget);
    encoder = SidsSmartCommand.radiansToEncoder(theta, 0.27, 0.55);
    // rotate robot

    // rotate hood
    // hood.setPositionRotations(encoder);
    System.out.println(encoder);
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
    return true;
  }
}
