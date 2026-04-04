package frc.robot.commands;

public class SidsSmartCommand {
  public static double getTargetDistance(
      double xrobot, double xtarget, double yrobot, double ytarget) {
    double xdis = xtarget - xrobot;
    double ydis = ytarget - yrobot;
    double distance = Math.sqrt(Math.pow(xdis, 2) + Math.pow(ydis, 2));
    return distance;
  }

  public static double getTargetAngle(
      double xrobot, double xtarget, double yrobot, double ytarget) {
    double xdis = xtarget - xrobot;
    double ydis = ytarget - yrobot;
    double angle = Math.atan2(ydis, xdis);
    return angle;
  }

  public static double getSpeedFromHoodAngle(
      double distance, double circumfrence, double hoodAngle, double gravity) {
    double speed = Math.sqrt((distance * gravity) / (circumfrence * Math.sin(2 * hoodAngle)));
    return speed;
  }

  public static double getHoodAnglefromSpeed(
      double distance, double circumfrence, double speed, double gravity) {
    double angle =
        0.5 * Math.asin((distance * gravity) / (circumfrence * circumfrence * speed * speed));
    return angle;
  }

  public static boolean verifySpeedAndHoodAngle(
      double circumfrence, double hoodAngle, double speed, double gravity, double height) {
    double maxHeight =
        (circumfrence * circumfrence * speed * speed * Math.sin(hoodAngle) * Math.sin(hoodAngle))
            / (2 * gravity);
    return height >= maxHeight;
  }
}
