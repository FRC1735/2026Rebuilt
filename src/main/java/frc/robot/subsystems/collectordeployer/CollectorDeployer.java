package frc.robot.subsystems.collectordeployer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// TODO - there is a ton of stuff in here that doesn't apply to current setup
// TODO - the entry point methods are doing the bang-bang approach from old collector
public class CollectorDeployer extends SubsystemBase {

  private final CollectorDeployerIO io;
  private final CollectorDeployerIOInputsAutoLogged inputs =
      new CollectorDeployerIOInputsAutoLogged();

  private static final double MIN_ROT = 0.2; // TODO
  private static final double MAX_ROT = 0.7; // TODO

  public static final double DEPLOYED_TARGET = 0.565; // TODO
  public static final double CLOSED_TARGET = 0.26; // TODO
  private static final double RANGE = 0.02; // TODO


  private final String name;

  public CollectorDeployer(CollectorDeployerIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public CollectorDeployerIO.CollectorState getCollectorState() {
    return inputs.state;
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

  public void setState(CollectorDeployerIO.CollectorState state) {
    io.setState(state);
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }

  public void deploy() {
    setState(CollectorDeployerIO.CollectorState.DEPLOYED);
  }

  public void close() {
    setState(CollectorDeployerIO.CollectorState.CLOSED);
  }

  double MANUAL_VOLTAGE = 7.5;
  double SLOW_VOLTAGE = 1;
  double EXTRA_SLOW_VOLTAGE = 0.7;

  static double DEPLOY_EXTRA_SLOW_RANGE = DEPLOYED_TARGET - 0.05;
  static double DEPLOY_SLOW_RANGE = DEPLOYED_TARGET - 0.1;
  static double DEPLOY_FULL_RANGE = CLOSED_TARGET;

  public void manualDeploy() {
    setVoltage(SLOW_VOLTAGE);
  }

  public void manualClose() {
    setVoltage(-SLOW_VOLTAGE);
  }

  // TODO - this logic already exists in the Close/Deploy commands, need to harmonize
  public void scaledDeploy() {
    double position = getPosition();

    if (position >= DEPLOY_EXTRA_SLOW_RANGE && position <= DEPLOYED_TARGET) {
      setVoltage(EXTRA_SLOW_VOLTAGE);
    } else if (position >= DEPLOY_SLOW_RANGE && position <= DEPLOYED_TARGET) {
      setVoltage(SLOW_VOLTAGE);
    } else if (position >= DEPLOY_FULL_RANGE) {
      setVoltage(MANUAL_VOLTAGE);
    }

  }

  public void scaleClose() {
    if (getPosition() < (CLOSED_TARGET + 0.5)) {
      setVoltage(-SLOW_VOLTAGE);
    } else if (getPosition() > CLOSED_TARGET) {
      setVoltage(-MANUAL_VOLTAGE);
    }
  }
}
