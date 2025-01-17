package frc.robot.Commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subSystems.Chassis;

public class Autonomous extends Command{
    private Chassis chassis;
    private Timer timer;
    public Autonomous(Chassis chassis)
    {
        this.chassis = chassis;
        addRequirements(chassis);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
    }
    

    @Override
    public void execute() {
        chassis.drive(0.5, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0);
    }
}
