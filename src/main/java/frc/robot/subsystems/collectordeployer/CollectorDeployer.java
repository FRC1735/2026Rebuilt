package frc.robot.subsystems.collectordeployer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CollectorDeployer extends SubsystemBase {

  private final CollectorDeployerIO io;
  private final CollectorDeployerIOInputsAutoLogged inputs =
      new CollectorDeployerIOInputsAutoLogged();

  private static final double MIN_ROT = 0; // TODO
  private static final double MAX_ROT = 1; // TODO

  private static final double DEPLOYED_TARGET = 0.5; // TODO
  private static final double RETRACTED_TARGET = 0.5; // TODO

  public CollectorDeployer(CollectorDeployerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Collector Deployer", inputs);
  }

  public void setTarget(double rotations) {
    double clamped = Math.max(MIN_ROT, Math.min(MAX_ROT, rotations));
    io.setTarget(clamped);
  }

  public void stop() {
    io.stop();
  }

  public boolean atTarget() {
    return inputs.atTarget;
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }

  public void deploy() {
    setTarget(DEPLOYED_TARGET);
  }

  public void retract() {
    setTarget(RETRACTED_TARGET);
  }

  public void incrementTargetPosition() {
    double position = inputs.targetRotations;
    double incremented = position + 0.01;
    if (incremented > 1) {
      incremented = 1;
    }
    setTarget(incremented);
  }

  public void decrementTargetPosition() {
    double position = inputs.targetRotations;
    double decremented = position - 0.01;
    if (decremented < 0) {
      decremented = 0;
    }
    setTarget(decremented);
  }
}
