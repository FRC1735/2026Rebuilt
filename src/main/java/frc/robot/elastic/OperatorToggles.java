package frc.robot.elastic;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OperatorToggles {
  private static final BooleanEntry enableVision =
      NetworkTableInstance.getDefault()
          .getTable("Elastic")
          .getBooleanTopic("EnableVision")
          .getEntry(false);

  public static boolean isVisionEnabled() {
    return enableVision.get();
  }
}
