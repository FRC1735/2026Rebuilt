package frc.robot.subsystems.collector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class CollectorIOSparkFlex implements CollectorIO {
 
    private final SparkFlex deploy;
  private final SparkFlex pull;
  private final AbsoluteEncoder deployAbsoluteEncoder;

  private final PIDController pid;
  private final SimpleMotorFeedforward feedforward;
  private static final String kPrefix = "Collector/";
  public CollectorIOSparkFlex (int deployCanId, int pullCanId) {
  
    deploy= new SparkFlex(deployCanId, MotorType.kBrushless);
    pull = new SparkFlex(pullCanId, MotorType.kBrushless);

    SparkFlexConfig deployConfig = new SparkFlexConfig();
    SparkFlexConfig pullConfig = new SparkFlexConfig();

    deployConfig.inverted(false).voltageCompensation(12.0).smartCurrentLimit(80);

    pullConfig.inverted(false).voltageCompensation(12.0).smartCurrentLimit(80);

    deploy.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pull.configure(
        pullConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deployAbsoluteEncoder = deploy.getAbsoluteEncoder();
}

}
