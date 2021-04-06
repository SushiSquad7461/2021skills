package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public class Logger {
    public static final String LOGGER_KEY = "log";
    
    private ArrayList<String> messages = new ArrayList<String>();

    private static Logger instance;

    private boolean shouldLogMessage(String msg) {
        if (this.messages.isEmpty()) return true;
        
        return !this.messages.get(this.messages.size() - 1).equals(msg);
    }

    private String formatAllMessages() {
        StringBuilder sb = new StringBuilder();

        for (String msg : this.messages) {
            sb.append(msg + "\n");
        }

        return sb.toString();
    }

    public static Logger getInstance() {
        if (instance == null) instance = new Logger();

        return instance;
    }

    private void logInternal(String msg) {
        if (!this.shouldLogMessage(msg)) return;
        this.messages.add(msg);
        SmartDashboard.putString(LOGGER_KEY, this.formatAllMessages());
        System.out.println(msg);
    }

    public static void log(String msg) {
        getInstance().logInternal(msg);
    }
}
