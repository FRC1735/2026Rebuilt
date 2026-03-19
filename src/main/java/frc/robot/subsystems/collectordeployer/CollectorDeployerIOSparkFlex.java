package frc.robot.subsystems.collectordeployer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.encoder.DetachedEncoder.Model;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CollectorDeployerIOSparkFlex implements CollectorDeployerIO {

  private final SparkFlex spark;
  private final DetachedEncoder encoder;
  private final PIDController pid;

  private double targetRotations = 0.0;

  private static final double HORIZONTAL_POSITION = 0.600;

  private static final double FORWARD_LIMIT = 0.7; // TODO
  private static final double REVERSE_LIMIT = 0.15; // TODO

  // Tunables
  private double kP = 06;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kG = 0.0001;

  private double lastP, lastI, lastD, lastG;

  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;
  private final DoubleEntry kGEntry;

  public CollectorDeployerIOSparkFlex(int motorCanId, int detachedEncoderCanId) {

    spark = new SparkFlex(motorCanId, MotorType.kBrushless);

    encoder = new DetachedEncoder(detachedEncoderCanId, Model.MAXSplineEncoder) {};

    NetworkTable table =
        NetworkTableInstance.getDefault().getTable("Elastic").getSubTable("Collector Deployer");

    kPEntry = table.getDoubleTopic("kP").getEntry(kP);
    kIEntry = table.getDoubleTopic("kI").getEntry(kI);
    kDEntry = table.getDoubleTopic("kD").getEntry(kD);
    kGEntry = table.getDoubleTopic("kG").getEntry(kG);

    kPEntry.set(kP);
    kIEntry.set(kI);
    kDEntry.set(kD);
    kGEntry.set(kG);

    pid = new PIDController(kP, kI, kD);
    pid.disableContinuousInput();
    pid.setTolerance(0.01);

    configureSpark();

    // make the current target whatever the encoder is when subsystem initialized
    targetRotations = encoder.getAngle();
  }

  private void configureSpark() {

    var config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    config.inverted(true);

    // NOTE: leaving this here, but these do not get applied because the motor is not aware of the
    // encoder
    config.softLimit.forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false);

    DetachedEncoderConfig encoderConfig = new DetachedEncoderConfig();
    encoderConfig.dutyCycleOffset(0);

    encoder.configure(encoderConfig, ResetMode.kNoResetSafeParameters);

    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void checkForChanges() {
    kP = kPEntry.get();
    kI = kIEntry.get();
    kD = kDEntry.get();
    kG = kGEntry.get(); // this will be applied in the kG calculation in updateInputs

    if (kP != lastP || kI != lastI || kD != lastD) {
      pid.setPID(kP, kI, kD);
    }
  }

  @Override
  public void updateInputs(CollectorDeployerIOInputs inputs) {

    checkForChanges();

    double position = encoder.getAngle();

    inputs.encoderPosition = position;
    inputs.velocityRPM = encoder.getVelocity();

    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();

    inputs.targetRotations = targetRotations;
    inputs.atTarget = pid.atSetpoint();

    inputs.kP = kP;
    inputs.kI = kI;
    inputs.kD = kD;
    inputs.kG = kG;

    inputs.allowedProfileError = 0;

    inputs.delta = position - targetRotations;

    // Gravity Feed Forward
    double error = position - HORIZONTAL_POSITION;
    if (error > 0.5) error -= 1;
    if (error < -0.5) error += 1;

    double positionRadians = error * 2 * Math.PI;
    double gravityFF = kG * Math.cos(positionRadians);

    double pidOutput = pid.calculate(position, targetRotations);
    double output = pidOutput + gravityFF;
    spark.set(output);

    // System.out.println("output: " + output);

    // System.out.println("radians: " + positionRadians);
    // System.out.println("cos: " + Math.cos(positionRadians));

    // TODO - reenable
    // spark.set(pid.calculate(encoder.getAngle(), targetRotations));
  }

  @Override
  public void setTarget(double target) {
    targetRotations = target;

    pid.reset();

    pid.setSetpoint(targetRotations);
  }

  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }
}
