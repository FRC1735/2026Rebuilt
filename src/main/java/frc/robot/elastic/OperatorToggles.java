package frc.robot.elastic;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OperatorToggles {
  private static final BooleanEntry enableVision =
      NetworkTableInstance.getDefault()
          .getTable("Elastic")
          .getBooleanTopic("EnableVision")
          .getEntry(true);

  public static void init() {
    enableVision.set(true);
  }

  public static boolean isVisionEnabled() {
    return enableVision.get();
  }

  public static void toggleVisionEnabled() {
    if (enableVision.get()) {
      enableVision.set(false);
    } else {
      enableVision.set(true);
    }
  }
}
