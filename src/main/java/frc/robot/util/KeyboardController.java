package frc.robot.util;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

public class KeyboardController {
  private final int numButtons;

  private NetworkTableInstance inst;
  private NetworkTable keyboardTable;
  BooleanSubscriber[] buttonSubscribers;
  BooleanSubscriber isConnectedSubscriber;

  private final Map<EventLoop, Map<Integer, Trigger>> m_buttonCache = new HashMap<>();

  // Group of triggers for high level subsystems
  private final Shooter shooter;
  private final Collector collector;
  private final Hood hood;
  private final Combo combo;

  public KeyboardController(int port) {
    this(port, 80);
  }

  public KeyboardController(int port, int numButtons) {
    this.numButtons = numButtons;
    this.inst = NetworkTableInstance.getDefault();
    this.keyboardTable = inst.getTable("/AdvantageKit/DriverStation/Keyboard" + port);

    this.shooter = new Shooter(this);
    this.collector = new Collector(this);
    this.combo = new Combo(this);
    this.hood = new Hood(this);

    buttonSubscribers = new BooleanSubscriber[this.numButtons];
    for (int i = 0; i < this.numButtons; i++) {
      buttonSubscribers[i] = keyboardTable.getBooleanTopic(String.valueOf(i)).subscribe(false);
    }
    isConnectedSubscriber = keyboardTable.getBooleanTopic("isConnected").subscribe(false);
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #button(int, EventLoop)
   */
  public Trigger button(int button) {
    return button(button, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the button's digital signal attached to the given loop.
   */
  public Trigger button(int button, EventLoop loop) {
    var cache = m_buttonCache.computeIfAbsent(loop, k -> new HashMap<>());
    return cache.computeIfAbsent(
        button, k -> new Trigger(loop, () -> getRawButton(k) && isConnected()));
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param row the row index
   * @param col the column index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #button(int, EventLoop)
   */
  public Trigger button(int row, int col) {
    return button(8 * (col - 1) + row - 1);
  }

  /**
   * Get if the HID is connected.
   *
   * @return true if the HID is connected
   */
  public boolean isConnected() {
    return isConnectedSubscriber.get();
  }

  private boolean getRawButton(int button) {
    return buttonSubscribers[button].get();
  }

  public Shooter shooter() {
    return shooter;
  }

  public Collector collector() {
    return collector;
  }

  public Hood hood() {
    return hood;
  }

  public Combo combo() {
    return combo;
  }

  public static final record Hood(KeyboardController controller) {
    public Trigger hoodToUpPosition() {
      return controller.button(7, 5);
    }

    public Trigger hoodToAutoPosition() {
      return controller.button(8, 5);
    }

    public Trigger hoodToDownPosition() {
      return controller.button(7, 6);
    }

    public Trigger manualMoveHoodUp() {
      return controller.button(2, 5);
    }

    public Trigger manualMoveHoodDown() {
      return controller.button(2, 6);
    }
  }

  public static final record Collector(KeyboardController controller) {
    public Trigger up() {
      return controller.button(7, 7);
    }

    public Trigger down() {
      return controller.button(7, 8);
    }

    public Trigger manualUp() {
      return controller.button(2, 7);
    }

    public Trigger manualDown() {
      return controller.button(2, 8);
    }

    public Trigger hardClose() {
      return controller.button(3, 8);
    }

    public Trigger in() {
      return controller.button(8, 7);
    }

    public Trigger out() {
      return controller.button(8, 8);
    }
  }

  public static final record Combo(KeyboardController controller) {
    public Trigger shootIntoHubAndCollect() {
      return controller
          .button(1, 2)
          .or(controller.button(1, 3))
          .or(controller.button(2, 2))
          .or(controller.button(2, 3));
    }

    public Trigger shootIntoHub() {
      return controller
          .button(3, 2)
          .or(controller.button(3, 3))
          .or(controller.button(4, 2))
          .or(controller.button(4, 3));
    }

    public Trigger passAndCollect() {
      return controller
          .button(5, 2)
          .or(controller.button(5, 3))
          .or(controller.button(6, 2))
          .or(controller.button(6, 3));
    }

    public Trigger collect() {
      return controller
          .button(7, 2)
          .or(controller.button(7, 3))
          .or(controller.button(8, 2))
          .or(controller.button(8, 3));
    }

    public Trigger storage() {
      return controller
          .button(4, 5)
          .or(controller.button(4, 6))
          .or(controller.button(4, 7))
          .or(controller.button(4, 8));
    }
  }

  public static final record Shooter(KeyboardController controller) {
    public Trigger shoot5600() {
      return controller.button(2, 10);
    }

    public Trigger shoot5000() {
      return controller.button(3, 10);
    }

    public Trigger shoot4500() {
      return controller.button(4, 10);
    }

    public Trigger shoot4000() {
      return controller.button(5, 10);
    }

    public Trigger shoot3500() {
      return controller.button(6, 10);
    }

    public Trigger shoot3000() {
      return controller.button(7, 10);
    }

    public Trigger shoot2500() {
      return controller.button(8, 10);
    }

    public Trigger shootAtDepot() {
      return controller.button(8, 9);
    }

    public Trigger reverseShooterAndShooterIntake() {
      return controller
          .button(1, 1)
          .or(controller.button(2, 1))
          .or(controller.button(3, 1))
          .or(controller.button(4, 1));
    }

    public Trigger reverseShooterIntake() {
      return controller
          .button(5, 1)
          .or(controller.button(6, 1))
          .or(controller.button(7, 1))
          .or(controller.button(8, 1));
    }
  }
}
