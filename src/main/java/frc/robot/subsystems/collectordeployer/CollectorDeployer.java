package frc.robot.subsystems.collectordeployer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CollectorDeployer extends SubsystemBase {

  private final CollectorDeployerIO io;
  private final CollectorDeployerIOInputsAutoLogged inputs =
      new CollectorDeployerIOInputsAutoLogged();

  private static final double MIN_ROT = 0; // TODO
  private static final double MAX_ROT = 1; // TODO

  public CollectorDeployer(CollectorDeployerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Collector Deployer", inputs);
  }

  public void setPositionRotations(double rotations) {
    double clamped = Math.max(MIN_ROT, Math.min(MAX_ROT, rotations));
    io.setPositionRotations(clamped);
  }

  public void stop() {
    io.stop();
  }

  public boolean atTarget() {
    return inputs.atTarget;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  // incrementPossition by 0.01
  public void incrementTargetPosition() {
    double position = inputs.targetRotations;
    double incremented = position + 0.01;
    if (incremented > 1) {
      incremented = 0;
    }
    setPositionRotations(incremented);
  }

  // decrementPosition by -0.01
  public void decrementTargetPosition() {
    double position = inputs.targetRotations;
    double decremented = position - 0.01;
    if (decremented < 0) {
      decremented = 1;
    }
    setPositionRotations(decremented);
  }
}
