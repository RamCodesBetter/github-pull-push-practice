
package frc.robot.Commands.Convayor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Conveyor.ConveyorStates;

public class SetConveyor extends Command {
    private Conveyor s_Conveyor;
    ConveyorStates state;

    public SetConveyor(ConveyorStates state) {
        s_Conveyor = Conveyor.getInstance();

        this.state = state;
        addRequirements(s_Conveyor);
    }

    @Override
    public void initialize() {
        s_Conveyor.setVelocity(state.getRps());
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
