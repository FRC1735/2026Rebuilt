package frc.robot.subsystems.collectordeployer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// TODO - there is a ton of stuff in here that doesn't apply to current setup
// TODO - the entry point methods are doing the bang-bang approach from old collector
public class CollectorDeployer extends SubsystemBase {

  private final CollectorDeployerIO io;
  private final CollectorDeployerIOInputsAutoLogged inputs =
      new CollectorDeployerIOInputsAutoLogged();

  private static final double MIN_ROT = 0.3; // TODO
  private static final double MAX_ROT = 0.7; // TODO

  private static final double DEPLOYED_TARGET = 0.48; // TODO
  private static final double CLOSED_TARGET = 0.55; // TODO
  private static final double RANGE = 0.02; // TODO

  private final String name;

  enum State {
    CLOSED,
    DEPLOYING,
    DEPLOYED
  }

  public CollectorDeployer(CollectorDeployerIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void setTarget(double rotations) {
    double clamped = Math.max(MIN_ROT, Math.min(MAX_ROT, rotations));
    io.setTarget(clamped);
  }

  public void stop() {
    io.stop();
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public boolean atTarget() {
    return inputs.atTarget;
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }

  public State getState() {
    if ((DEPLOYED_TARGET - RANGE) <= inputs.encoderPosition
        && inputs.encoderPosition <= (RANGE + DEPLOYED_TARGET)) {
      return State.DEPLOYED;
    } else if ((CLOSED_TARGET - RANGE) <= inputs.encoderPosition
        && inputs.encoderPosition <= (RANGE + CLOSED_TARGET)) {
      return State.CLOSED;
    }
    return State.DEPLOYING;
  }

  public void deploy() {
    setTarget(DEPLOYED_TARGET);
  }

  public void close() {
    setTarget(CLOSED_TARGET);
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

  double MANUAL_VOLTAGE = 1.5;

  public void manualDeploy() {
    setVoltage(-1.5);
  }

  public void manualClose() {
    setVoltage(3.2);
  }
}
