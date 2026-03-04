package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;

public class TunableNumber {
  private final DoubleEntry entry;
  private double lastValue;

  public TunableNumber(NetworkTable table, String key, double defaultValue) {
    entry = table.getDoubleTopic(key).getEntry(defaultValue);
    entry.set(defaultValue);
    lastValue = defaultValue;
  }

  public double get() {
    return entry.get();
  }

  public boolean hasChanged(double tolerance) {
    double current = get();
    if (Math.abs(current - lastValue) > tolerance) {
      lastValue = current;
      return true;
    }
    return false;
  }
}
