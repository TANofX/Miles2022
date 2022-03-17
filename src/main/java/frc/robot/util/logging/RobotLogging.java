// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class RobotLogging {
    private class LogValue<K> {
        protected String keyValue;
        private Supplier<K> valueSupplier;
        private boolean checkHistory;
        protected K currentValue = null;

        protected LogValue(String key, Supplier<K> supplier, boolean checkHistory) {
            keyValue = key;
            valueSupplier = supplier;
            this.checkHistory = checkHistory;
        }

        protected void log() {
            if (checkHistory) {
                if (!getValue().equals(currentValue)) {
                    currentValue = getValue();
                    logToSmartdashboard();
                }
            } else {
                currentValue = getValue();
                logToSmartdashboard();
            }
        }

        protected void logToSmartdashboard() {
            if (currentValue instanceof Integer) {
                SmartDashboard.putNumber(keyValue, ((Integer)currentValue).doubleValue());
            } else if (currentValue instanceof Double) {
                SmartDashboard.putNumber(keyValue, ((Double)currentValue).doubleValue());
            } else if (currentValue instanceof Boolean) {
                SmartDashboard.putBoolean(keyValue, ((Boolean)currentValue).booleanValue());
            } else {
                SmartDashboard.putString(keyValue, getStringValue());
            }
        }

        protected K getValue() {
            return valueSupplier.get();
        }

        protected String getStringValue() {
            return currentValue.toString();
        }
    }

    private class LogEnumValue extends LogValue<Enum<?>> {
        protected LogEnumValue(String key, Supplier<Enum<?>> supplier, boolean checkHistory) {
            super(key, supplier, checkHistory);
        }

        protected String getStringValue() {
            return currentValue.name();
        }
    }

    private static RobotLogging singleInstance;
    private List<LogValue<?>> logList = new ArrayList<>();

    protected RobotLogging() {}

    public static RobotLogging getInstance() {
        if (singleInstance == null) {
            singleInstance = new RobotLogging();
        }

        return singleInstance;
    }

    private void registerLogger(LogValue<?> newLog) {
        logList.add(newLog);
    }

    public void registerImmediate(String key, Supplier<Double> valueSupplier) {
        registerLogger(new LogValue<Double>(key, valueSupplier, false));
    }

    public void registerHistorian(String key, Supplier<Double> valueSupplier) {
        registerLogger(new LogValue<Double>(key, valueSupplier, true));
    }

    public void registerEnumImmediate(String key, Supplier<Enum<?>> valueSupplier) {
        registerLogger(new LogEnumValue(key, valueSupplier, false));
    }

    public void registerEnumHistorian(String key, Supplier<Enum<?>> valueSupplier) {
        registerLogger(new LogEnumValue(key, valueSupplier, true));
    }

    public void logValues() {
        for (LogValue<?> a: logList) {
            a.log();
        }
    }
}
