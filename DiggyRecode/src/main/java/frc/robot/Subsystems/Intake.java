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
 * Intake subsystem — the roller that pulls balls from the floor into the robot.
 * Speed is closed-loop rotations per second. The intake pivot is a separate
 * subsystem ({@link Pivot}) and stays on voltage.
 *
 * <p>
 * SysId routines are preserved in comments below for future characterization if
 * needed.
 */
public class Intake extends SubsystemBase {
    private static Intake instance;

    /** Reused so each setVelocity() does not allocate. Slot 0 holds the gains below. */
    private final VelocityVoltage rpsRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    /**
     * Returns the singleton instance of Intake, creating it if necessary.
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    /**
     * Named speeds in motor rotations per second. Positive = intake direction.
     * Old voltages: ON 5 V, CYCLE 2 V, REVERSE -2 V. Divided by kV 0.12. Retune on the robot.
     */
    public enum IntakeStates {
        ON(42),
        CYCLE(17),
        OFF(0),
        REVERSE(-17);

        private final double rps;

        private IntakeStates(double rps) {
            this.rps = rps;
        }

        /** Returns the target speed for this state, in rotations per second. */
        public double getRps() {
            return rps;
        }
    }

    private final TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(Constants.HardwarePorts.intake, Constants.mechBus);
        configureMotor(intakeMotor, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
    }

    /**
     * Configures a TalonFX motor with neutral mode, inversion, velocity gains,
     * stator current limit, and optimized CAN bus utilization.
     *
     * @param motor       The TalonFX to configure
     * @param neutralMode Brake or Coast when no output is applied
     * @param direction   Motor inversion direction
     */
    private void configureMotor(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = direction;
        config.MotorOutput.NeutralMode = neutralMode;

        config.Slot0.kS = Constants.RollerGains.kS;
        config.Slot0.kV = Constants.RollerGains.kV;
        config.Slot0.kP = Constants.RollerGains.kP;

        // Stator limit prevents the intake motor from burning out during hard jams
        config.CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.intakeStator;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);

        // Status frames to the Rio only. The velocity loop runs on the motor.
        motor.getPosition().setUpdateFrequency(4);
        motor.getVelocity().setUpdateFrequency(4);

        // Suppress unused status frames to reduce CAN bus utilization
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the intake roller speed in rotations per second.
     * Zero releases the motor to its neutral mode (Brake) instead of having
     * the PID hold 0 rps.
     *
     * @param rps Target speed (positive = intake direction)
     */
    public void setVelocity(double rps) {
        if (rps == 0) {
            intakeMotor.setControl(neutralRequest);
        } else {
            intakeMotor.setControl(rpsRequest.withVelocity(rps));
        }
    }

    /**
     * Returns a one-shot command that applies the given intake state.
     *
     * @param state The desired IntakeState (ON, CYCLE, OFF, REVERSE)
     * @return An InstantCommand-style command
     */
    public Command setState(IntakeStates state) {
        return Commands.runOnce(() -> setVelocity(state.getRps()), this);
    }

    @Override
    public void periodic() {
        // No periodic telemetry needed for the intake at this time
    }

    // ---- SysId routines (preserved for future characterization, not currently
    // active) ----

    // private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(
    // Volts.of(1).div(Seconds.of(1)),
    // Volts.of(8),
    // Units.Seconds.of(8),
    // (state) -> SignalLogger.writeString("SysIdIntake_State", state.toString())
    // ),
    // new SysIdRoutine.Mechanism(
    // (volts) -> setVoltage(volts.in(Volts)),
    // log -> {log.motor("intake_motor")
    // .voltage(intakeMotor.getMotorVoltage().getValue())
    // .angularPosition(intakeMotor.getPosition().getValue())
    // .angularVelocity(intakeMotor.getVelocity().getValue()); },
    // this
    // ));

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.dynamic(direction);
    // }
}
