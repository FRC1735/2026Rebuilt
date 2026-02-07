package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOSparkFlex implements ShooterIO {

  private final SparkFlex leader;
  private final SparkFlex follower;
  private final RelativeEncoder leaderEncoder;

  private final PIDController pid;
  private final SimpleMotorFeedforward feedforward;

  // Dashboard keys
  private static final String kPrefix = "Shooter/";

  public ShooterIOSparkFlex(int leaderCanId, int followerCanId) {

    leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
    follower = new SparkFlex(followerCanId, MotorType.kBrushless);

    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    SparkFlexConfig followerConfig = new SparkFlexConfig();

    leaderConfig.inverted(false).voltageCompensation(12.0).smartCurrentLimit(80);

    followerConfig.follow(leader, false).voltageCompensation(12.0).smartCurrentLimit(80);

    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leaderEncoder = leader.getEncoder();

    // ---------------- CONTROL ----------------

    pid =
        new PIDController(
            SmartDashboard.getNumber(kPrefix + "kP", 0.0005),
            SmartDashboard.getNumber(kPrefix + "kI", 0.0),
            SmartDashboard.getNumber(kPrefix + "kD", 0.0));

    pid.setTolerance(SmartDashboard.getNumber(kPrefix + "VelocityToleranceRadPerSec", 5.0));

    feedforward =
        new SimpleMotorFeedforward(
            SmartDashboard.getNumber(kPrefix + "kS", 0.25),
            SmartDashboard.getNumber(kPrefix + "kV", 0.12),
            SmartDashboard.getNumber(kPrefix + "kA", 0.0));

    // Seed dashboard values (Elastic-friendly)
    SmartDashboard.setDefaultNumber(kPrefix + "kP", 0.0005);
    SmartDashboard.setDefaultNumber(kPrefix + "kI", 0.0);
    SmartDashboard.setDefaultNumber(kPrefix + "kD", 0.0);
    SmartDashboard.setDefaultNumber(kPrefix + "kS", 0.25);
    SmartDashboard.setDefaultNumber(kPrefix + "kV", 0.12);
    SmartDashboard.setDefaultNumber(kPrefix + "kA", 0.0);
    SmartDashboard.setDefaultNumber(kPrefix + "TargetRadPerSec", 0.0);
    SmartDashboard.setDefaultNumber(kPrefix + "VelocityToleranceRadPerSec", 5.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    double velocityRadPerSec = leaderEncoder.getVelocity() * (2.0 * Math.PI / 60.0);

    inputs.leftVelocityRadPerSec = velocityRadPerSec;
    inputs.rightVelocityRadPerSec = velocityRadPerSec;

    inputs.leftAppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.rightAppliedVolts = follower.getAppliedOutput() * follower.getBusVoltage();

    inputs.leftCurrentAmps = leader.getOutputCurrent();
    inputs.rightCurrentAmps = follower.getOutputCurrent();

    inputs.leftConnected = leader.getLastError() == REVLibError.kOk;
    inputs.rightConnected = follower.getLastError() == REVLibError.kOk;

    // ---------------- CONTROL UPDATE ----------------

    // Pull tunables live (Elastic edits apply immediately)
    pid.setPID(
        SmartDashboard.getNumber(kPrefix + "kP", pid.getP()),
        SmartDashboard.getNumber(kPrefix + "kI", pid.getI()),
        SmartDashboard.getNumber(kPrefix + "kD", pid.getD()));

    pid.setTolerance(
        SmartDashboard.getNumber(kPrefix + "VelocityToleranceRadPerSec", pid.getErrorTolerance()));

    double targetRadPerSec = SmartDashboard.getNumber(kPrefix + "TargetRadPerSec", 0.0);

    double ffVolts = feedforward.calculate(targetRadPerSec);

    double pidVolts = pid.calculate(velocityRadPerSec, targetRadPerSec);

    leader.setVoltage(ffVolts + pidVolts);
  }

  @Override
  public void setVoltage(double volts, double ignored) {
    leader.setVoltage(volts);
    pid.reset();
  }

  @Override
  public void stop() {
    leader.stopMotor();
    pid.reset();
  }
}
