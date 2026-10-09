package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Indexer subsystem — feeds balls from the conveyor into the shooter.
 * Speed is closed-loop rotations per second on the TalonFX.
 */
public class Indexer extends SubsystemBase {
    private static Indexer instance;

    /**
     * Returns the singleton instance of Indexer, creating it if necessary.
     * All subsystems use singletons so hardware is only configured once.
     */
    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    /**
     * Named speeds in motor rotations per second. Positive = toward the shooter.
     * Old voltages: ON 6 V, REVERSE -3 V. Divided by kV 0.12. Retune on the robot.
     */
    public enum IndexerStates {
        ON(50),
        OFF(0),
        REVERSE(-25);

        private final double rps;

        private IndexerStates(double rps) {
            this.rps = rps;
        }

        /** Returns the target speed for this state, in rotations per second. */
        public double getRps() {
            return rps;
        }
    }

    private final TalonFX indexerMotor;

    /** Reused so each setVelocity() does not allocate. Slot 0 holds the gains below. */
    private final VelocityVoltage rpsRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public Indexer() {
        indexerMotor = new TalonFX(Constants.HardwarePorts.indexer, Constants.mechBus);
        config(indexerMotor, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    }

    /**
     * Configures a TalonFX motor with neutral mode, direction, velocity gains,
     * current limits, and optimized CAN bus utilization.
     *
     * @param motor       The TalonFX motor to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void config(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = direction;

        config.Slot0.kS = Constants.RollerGains.kS;
        config.Slot0.kV = Constants.RollerGains.kV;
        config.Slot0.kP = Constants.RollerGains.kP;

        // Stator current limit protects the motor from overheating under load
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.indexerStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Status frames to the Rio only. The velocity loop runs on the motor.
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress all unused status signals to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the indexer speed in rotations per second.
     * Zero releases the motor to its neutral mode (Brake) instead of having
     * the PID hold 0 rps.
     *
     * @param rps Target speed (positive = toward shooter)
     */
    public void setVelocity(double rps) {
        if (rps == 0) {
            indexerMotor.setControl(neutralRequest);
        } else {
            indexerMotor.setControl(rpsRequest.withVelocity(rps));
        }
    }

    /**
     * Returns a one-shot command that sets the indexer to the given state.
     *
     * @param state The desired IndexerState (ON, OFF, REVERSE)
     * @return An InstantCommand-style command that applies the speed
     */
    public Command setState(IndexerStates state) {
        return Commands.runOnce(() -> setVelocity(state.getRps()), this);
    }

    @Override
    public void periodic() {
    }
}
