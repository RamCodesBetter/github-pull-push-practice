
package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.IntakeStates;

public class SetIntake extends Command {
    private Intake s_Intake;
    IntakeStates state;


    // Overloaded Constructor
    public SetIntake(IntakeStates state) {
        s_Intake = Intake.getInstance();

        this.state = state;

        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        s_Intake.setVelocity(state.getRps());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
