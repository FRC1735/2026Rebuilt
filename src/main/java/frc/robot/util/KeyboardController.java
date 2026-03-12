package frc.robot.util;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.Shooter;
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
  private final LeftCollector leftCollector;
  private final RightCollector rightCollector;

  public KeyboardController(int port) {
    this(port, 80);
  }

  public KeyboardController(int port, int numButtons) {
    this.numButtons = numButtons;
    this.inst = NetworkTableInstance.getDefault();
    this.keyboardTable = inst.getTable("/AdvantageKit/DriverStation/Keyboard" + port);

    this.shooter = new Shooter(this);
    this.leftCollector = new LeftCollector(this);
    this.rightCollector = new RightCollector(this);

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

  public LeftCollector collector() {
    return leftCollector;
  }


  public static final record Shooter(KeyboardController controller) {
    public Trigger hoodToUpPosition() {
      return controller.button(7, 1).or(controller.button(7, 2));
    }

    public Trigger manualMoveHoodUp() {
      return controller.button(7, 3);
    }

    public Trigger shooterHighSpeed() {
      return controller.button(7, 4);
    }

    public Trigger hoodToDownPosition() {
      return controller.button(8, 1).or(controller.button(8, 2));
    }

    public Trigger manualMoveHoodDown() {
      return controller.button(8, 3);
    }

    public Trigger shooterLowSpeed() {
      return controller.button(8, 4);
    }

    public Trigger shooterRollerIn() {
      return controller.button(7, 5);
    }

    public Trigger shooterRollerOut() {
      return controller.button(8, 5);
    }
  }

  public static final record LeftCollector(KeyboardController controller) {
    public Trigger deploy() {
      return controller.button(2, 6).or(controller.button(2, 7));
    }

    public Trigger retract() {
      return controller.button(3, 6).or(controller.button(3, 7));
    }

    public Trigger intake() {
      return controller.button(4, 6).or(controller.button(4, 7));
    }

    public Trigger outtake() {
      return controller.button(5, 6).or(controller.button(5, 7));
    }

    public Trigger manualDeploy() {
      return controller.button(6, 6).or(controller.button(6, 7));
    }

    public Trigger manualRetract() {
      return controller.button(7, 6).or(controller.button(7, 7));
    }
  }

  public static final record RightCollector(KeyboardController controller) {
    public Trigger deploy() {
      return controller.button(2, 9).or(controller.button(2, 10));
    }

    public Trigger retract() {
      return controller.button(3, 9).or(controller.button(3, 10));
    }

    public Trigger intake() {
      return controller.button(4, 9).or(controller.button(4, 10));
    }

    public Trigger outtake() {
      return controller.button(5, 9).or(controller.button(5, 10));
    }

    public Trigger manualDeploy() {
      return controller.button(6, 9).or(controller.button(6, 10));
    }

    public Trigger manualRetract() {
      return controller.button(7, 9).or(controller.button(7, 10));
    }
  }
}
