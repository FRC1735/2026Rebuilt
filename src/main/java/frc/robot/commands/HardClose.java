// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;
import frc.robot.subsystems.collectordeployer.CollectorDeployerIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HardClose extends Command {
  private final CollectorDeployer collectorDeployer;

  /** Creates a new DeployCollector. */
  public HardClose(CollectorDeployer collectorDeployer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.collectorDeployer = collectorDeployer;
    addRequirements(collectorDeployer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collectorDeployer.setState(CollectorDeployerIO.CollectorState.CLOSED);
    collectorDeployer.hardClose();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectorDeployer.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double position = collectorDeployer.getPosition();
    return position < CollectorDeployer.CLOSED_TARGET + 0.005;
  }
}
