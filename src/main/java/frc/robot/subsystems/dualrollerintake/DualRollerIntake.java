package frc.robot.subsystems.dualrollerintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DualRollerIntake extends SubsystemBase {
  private final DualRollerIntakeIO io;
  private final DualRollerIntakeIOInputsAutoLogged inputs =
      new DualRollerIntakeIOInputsAutoLogged();
  private final String name;

  public DualRollerIntake(DualRollerIntakeIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(name, inputs);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void in() {
    io.setSpeed(1);
  }

  public void out() {
    io.setSpeed(-1);
  }

  public void stop() {
    io.stop();
  }
}
