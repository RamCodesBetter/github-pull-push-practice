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
 * Conveyor subsystem — moves balls from the intake zone into the indexer.
 * Speed is closed-loop rotations per second. Runs in Coast mode so balls don't jam on stop.
 */
public class Conveyor extends SubsystemBase {
    private static Conveyor instance;

    /**
     * Returns the singleton instance of Conveyor, creating it if necessary.
     */
    public static Conveyor getInstance() {
        if (instance == null) {
            instance = new Conveyor();
        }
        return instance;
    }

    /** Reused so each setVelocity() does not allocate. Slot 0 holds the gains below. */
    private final VelocityVoltage rpsRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    /**
     * Named speeds in motor rotations per second. Positive = toward the indexer.
     * Old voltages: ON 4.5 V, CYCLE 3 V, REVERSE -3 V. Divided by kV 0.12. Retune on the robot.
     */
    public enum ConveyorStates {
        ON(37.5),
        CYCLE(25),
        OFF(0),
        REVERSE(-25);

        private final double rps;

        private ConveyorStates(double rps) {
            this.rps = rps;
        }

        /** Returns the target speed for this state, in rotations per second. */
        public double getRps() {
            return rps;
        }
    }

    private final TalonFX conveyorMotor;

    public Conveyor() {
        conveyorMotor = new TalonFX(Constants.HardwarePorts.conveyor, Constants.mechBus);
        config(conveyorMotor, NeutralModeValue.Coast, InvertedValue.Clockwise_Positive);
    }

    /**
     * Configures a TalonFX motor with neutral mode, inversion, velocity gains,
     * current limits, and optimized CAN bus utilization.
     *
     * @param motor       The TalonFX to configure
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

        // Supply + stator current limits to protect motor and PDP
        config.CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.conveyorSupply;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.conveyorStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Status frames to the Rio only. The velocity loop runs on the motor.
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress unused status frames to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the conveyor speed in rotations per second.
     * Zero releases the motor so it actually coasts; a PID holding 0 rps would brake it.
     *
     * @param rps Target speed (positive = toward indexer)
     */
    public void setVelocity(double rps) {
        if (rps == 0) {
            conveyorMotor.setControl(neutralRequest);
        } else {
            conveyorMotor.setControl(rpsRequest.withVelocity(rps));
        }
    }

    /**
     * Returns a one-shot command that applies the given conveyor state.
     *
     * @param state The desired ConveyorState
     * @return An InstantCommand-style command
     */
    public Command setState(ConveyorStates state) {
        return Commands.runOnce(() -> setVelocity(state.getRps()), this);
    }

    @Override
    public void periodic() {
        // No periodic telemetry needed for the conveyor at this time
    }
}
