package frc.robot.subsystems.ShooterHood;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterHoodIOSparkFlex implements ShooterHoodIO {

  private final SparkFlex motor;
  private final AbsoluteEncoder encoder;

  private final PIDController pid;

  // Dashboard keys
  private static final String kPrefix = "ShooterHood/";

  public ShooterHoodIOSparkFlex(int motorCanId) {

    motor = new SparkFlex(motorCanId, MotorType.kBrushless);

    SparkFlexConfig motorConfig = new SparkFlexConfig();

    motorConfig.inverted(false).voltageCompensation(12.0).smartCurrentLimit(80);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    pid =
        new PIDController(
            SmartDashboard.getNumber(kPrefix + "kP", 0.0005),
            SmartDashboard.getNumber(kPrefix + "kI", 0.0),
            SmartDashboard.getNumber(kPrefix + "kD", 0.0));

    pid.setTolerance(SmartDashboard.getNumber(kPrefix + "VelocityToleranceRadPerSec", 5.0));

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
}
