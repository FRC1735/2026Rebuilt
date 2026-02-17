package frc.robot.subsystems.shooterhood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class ShooterHoodIOSim implements ShooterHoodIO {

  private double position = 0.0;
  private double velocity = 0.0;
  private double target = 0.0;

  private double cruiseVelocity = 2.0;
  private double acceleration = 4.0;
  private double allowedProfileError = 0.01;

  private double lastTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {

    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    double error = target - position;

    double desiredVel = MathUtil.clamp(error * 5.0, -cruiseVelocity, cruiseVelocity);

    double accel = MathUtil.clamp(desiredVel - velocity, -acceleration * dt, acceleration * dt);

    velocity += accel;
    position += velocity * dt;

    if (Math.abs(error) < allowedProfileError) {
      position = target;
      velocity = 0.0;
    }

    inputs.encoderPosition = position;
    inputs.velocityRPM = velocity * 60.0;
    inputs.appliedVolts = MathUtil.clamp(error * 6.0, -12, 12);
    inputs.currentAmps = Math.abs(inputs.appliedVolts) * 2.0;

    inputs.targetRotations = target;
    inputs.atTarget = Math.abs(error) < allowedProfileError;

    inputs.cruiseVelocity = cruiseVelocity;
    inputs.acceleration = acceleration;
    inputs.allowedProfileError = allowedProfileError;
  }

  @Override
  public void setTarget(double rotations) {
    target = rotations;
  }

  @Override
  public void stop() {
    velocity = 0.0;
  }
}
