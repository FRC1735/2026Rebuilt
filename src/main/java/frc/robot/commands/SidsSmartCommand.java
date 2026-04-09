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

  public static double getHoodAngle(
      double xrobot,
      double xtarget,
      double yrobot,
      double ytarget,
      double height,
      double circumfrence,
      double rps,
      double gravity) {
    double distance = getTargetDistance(xrobot, xtarget, yrobot, ytarget);
    double linearSpeed = rps * circumfrence;
    double theta = calculateProjectileMotion(distance, height, linearSpeed, gravity);
    return theta;
  }

  public static double calculateProjectileMotion(double d, double h, double u, double g) {
    double discriminant = Math.pow(u, 4) - g * (g * d * d + 2 * h * u * u);
    if (discriminant < 0) {
      return -1;
    }
    double theta1 = Math.atan((u * u + Math.sqrt(discriminant)) / (g * d));
    double theta2 = Math.atan((u * u - Math.sqrt(discriminant)) / (g * d));
    if (theta1 < Math.PI / 6 || theta1 > Math.PI / 2) {
      return -1;
    }
    return theta1;
  }

  public static double radiansToEncoder(double radians, double a, double b) {
    if (radians == -1) {
      return 0;
    }
    double dis = b - a;
    double dec = (radians - (Math.PI / 6)) / (Math.PI / 3);
    return (dis * dec + a);
  }
}
