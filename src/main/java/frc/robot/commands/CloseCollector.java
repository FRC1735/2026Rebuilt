// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collectordeployer.CollectorDeployer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CloseCollector extends Command {
  private final CollectorDeployer collectorDeployer;

  /** Creates a new DeployCollector. */
  public CloseCollector(CollectorDeployer collectorDeployer) {
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
    double position = collectorDeployer.getPosition();

    // go slow
    boolean goSlow =
        position > CollectorDeployer.CLOSED_TARGET
            && position < CollectorDeployer.CLOSED_TARGET + 0.15;
    if (goSlow) {
      collectorDeployer.setVoltage(-1);
    } else if (position > CollectorDeployer.CLOSED_TARGET) {
      collectorDeployer.setVoltage(-7.5);
    }
  }
  /*     double t =
      (position - CollectorDeployer.CLOSED_TARGET)
          / (CollectorDeployer.DEPLOYED_TARGET - CollectorDeployer.CLOSED_TARGET);
  double voltage = -(t * 6.5 + 1);
  collectorDeployer.setVoltage(voltage); */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectorDeployer.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double position = collectorDeployer.getPosition();
    return position < CollectorDeployer.CLOSED_TARGET + 0.01;
  }
}
