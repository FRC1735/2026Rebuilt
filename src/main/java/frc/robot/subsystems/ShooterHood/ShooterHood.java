package frc.robot.subsystems.shooterhood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterHood extends SubsystemBase {

    private final ShooterHoodIO io;
    private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

    private static final double MIN_ROT = 0; // TODO
    private static final double MAX_ROT = 1; // TODO

    private static final double TARGET_UP = 0.5; // TODO
    private static final double TARGET_DOWN = 0.5; // TODO

    public ShooterHood(ShooterHoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterHood", inputs);
    }

    public void setPositionRotations(double rotations) {
        double clamped = Math.max(MIN_ROT, Math.min(MAX_ROT, rotations));
        io.setTarget(clamped);
    }

    public void setPositionUp() {
        setPositionRotations(TARGET_UP);
    }

    public void setPositionDown() {
        setPositionRotations(TARGET_DOWN);
    }

    public void incrementTargetPosition() {
        double position = inputs.targetRotations;
        double incremented = position + 0.01;
        if (incremented > 1) {
            incremented = 1;
        }
        io.setTarget(incremented);
    }

    public void decrementTargetPosition() {
        double position = inputs.targetRotations;
        double decremented = position - 0.01;
        if (decremented < 0) {
            decremented = 0;
        }
        io.setTarget(decremented);
    }

    public void stop() {
        io.stop();
    }

    public boolean atTarget() {
        return inputs.atTarget;
    }

    public double getPosition() {
        return inputs.encoderPosition;
    }
}

