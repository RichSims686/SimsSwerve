package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class PrintFormattedCommand extends InstantCommand {
    
    @FunctionalInterface
    public static interface ObjectSupplier extends Supplier<Object> {}
    
    private final String formattedMessage;
    private final ObjectSupplier[] objectSupplier;

    public PrintFormattedCommand(String formattedMessage, ObjectSupplier... objectSupplier) {
        this.formattedMessage = formattedMessage;
        this.objectSupplier = objectSupplier;
    }

    public void execute() {
        Object[] params = new Object[this.objectSupplier.length];
        for (int i=0; i<this.objectSupplier.length; i++) {
            params[i] = this.objectSupplier[i].get();
        }
        System.out.printf(formattedMessage, params);
    }
}
      
