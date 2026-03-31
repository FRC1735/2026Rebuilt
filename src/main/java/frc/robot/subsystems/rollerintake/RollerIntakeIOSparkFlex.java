package frc.robot.subsystems.rollerintake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RollerIntakeIOSparkFlex implements RollerIntakeIO {
  private final SparkFlex leader;
  private final SparkFlex follower;

  // Tunables
  private double kP = 4.0;
  private double kI = 0.0;
  private double kD = 5.0;
  private double cruiseVelocity = 500.0;
  private double acceleration = 200.0;
  private double allowedProfileError = 0.01;

  private double lastP, lastI, lastD;
  private double lastCruise, lastAccel, lastError;

  private boolean inverted;

  private final DoubleEntry kPEntry;
  private final DoubleEntry kIEntry;
  private final DoubleEntry kDEntry;
  private final DoubleEntry cruiseEntry;
  private final DoubleEntry accelEntry;
  private final DoubleEntry errorEntry;

  public RollerIntakeIOSparkFlex(
      int leaderCanId, int followerCanId, String name, boolean inverted) {

    leader = new SparkFlex(leaderCanId, MotorType.kBrushless);
    follower = new SparkFlex(followerCanId, MotorType.kBrushless);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(name);

    kPEntry = table.getDoubleTopic("kP").getEntry(kP);
    kIEntry = table.getDoubleTopic("kI").getEntry(kI);
    kDEntry = table.getDoubleTopic("kD").getEntry(kD);
    cruiseEntry = table.getDoubleTopic("cruiseVelocity").getEntry(cruiseVelocity);
    accelEntry = table.getDoubleTopic("acceleration").getEntry(acceleration);
    errorEntry = table.getDoubleTopic("allowedProfileError").getEntry(allowedProfileError);

    kPEntry.set(kP);
    kIEntry.set(kI);
    kDEntry.set(kD);
    cruiseEntry.set(cruiseVelocity);
    accelEntry.set(acceleration);
    errorEntry.set(allowedProfileError);

    this.inverted = inverted;

    configureSpark();
  }

  private void configureSpark() {

    var leaderconfig = new SparkFlexConfig();
    leaderconfig.idleMode(IdleMode.kCoast);
    leaderconfig.smartCurrentLimit(30);

    var followerConfig = new SparkFlexConfig();
    followerConfig.follow(leader, inverted);
    follower.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leaderconfig.closedLoop.outputRange(-1, 1).p(kP).i(kI).d(kD);

    leader.configure(
        leaderconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    lastP = kP;
    lastI = kI;
    lastD = kD;
    lastCruise = cruiseVelocity;
    lastAccel = acceleration;
    lastError = allowedProfileError;
  }

  private void checkForChanges() {

    kP = kPEntry.get();
    kI = kIEntry.get();
    kD = kDEntry.get();
    cruiseVelocity = cruiseEntry.get();
    acceleration = accelEntry.get();
    allowedProfileError = errorEntry.get();

    if (kP != lastP
        || kI != lastI
        || kD != lastD
        || cruiseVelocity != lastCruise
        || acceleration != lastAccel
        || allowedProfileError != lastError) {

      configureSpark();
    }
  }

  @Override
  public void updateInputs(RollerIntakeIOInputs inputs) {

    checkForChanges();

    inputs.velocity = leader.getEncoder().getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = leader.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    leader.set(volts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
