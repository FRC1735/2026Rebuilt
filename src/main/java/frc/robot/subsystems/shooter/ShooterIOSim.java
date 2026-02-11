package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {

  private double topVolts = 0.0;
  private double bottomVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftVelocityRadPerSec = topVolts * 40.0; // fake response
    inputs.rightVelocityRadPerSec = bottomVolts * 40.0;

    inputs.leftAppliedVolts = topVolts;
    inputs.rightAppliedVolts = bottomVolts;

    inputs.leftCurrentAmps = Math.abs(topVolts) * 5.0;
    inputs.rightCurrentAmps = Math.abs(bottomVolts) * 5.0;

    inputs.leftConnected = true;
    inputs.rightConnected = true;
  }

  @Override
  public void setVoltage(double topVolts, double bottomVolts) {
    this.topVolts = topVolts;
    this.bottomVolts = bottomVolts;
  }
}
