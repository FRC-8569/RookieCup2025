package frc.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Scoring {
    public static Scoring scoring;

    public int score = 0;
    public int Algaes = 0;
    public int Corals = 0;
    public int processor = 0;
    public int railing = 0;
    public boolean hasCoralBonus = false;
    public boolean hasAlgaeBonus = false; 
    public boolean hasProcessorBonus = false;
    public boolean hasL3 = false;
    public Alert CoralRPAlert = new Alert("System Alerts", "Coral RP Get", AlertType.kInfo);
    public Alert AlgaeAlert = new Alert("System Alerts", "Algae Bonus Get(6pt)", AlertType.kInfo);
    public Alert CoralAlert = new Alert("System Alerts", "Coral Bonus Get(6pt)", AlertType.kInfo);
    
    public void scoreRaw(FieldObject object){
        if(DriverStation.isAutonomous() || DriverStationSim.getAutonomous()){
            score += switch(object){
                case L1 -> 3;
                case L2 -> 5;
                case L3 -> 7; 
                case Railing -> 5;
                case Processor -> 4;
            };
        }else{
            score += switch(object){
                case L1 -> 2;
                case L2 -> 4;
                case L3 -> 6; 
                case Railing -> 5;
                case Processor -> 4;
            };
        }

        switch (object) {
            case L1 -> Corals++;
            case L2 -> Corals++;
            case L3 -> Corals++;
            case Railing ->{processor++; Algaes++;}
            case Processor -> Algaes++;
        }
        
        if(processor > 4 && !hasProcessorBonus){score += 4; hasProcessorBonus = true;}; 
        if(railing > 3 && Algaes > 6 && !hasAlgaeBonus){ score += 6; hasAlgaeBonus = true;};  
        if(Corals > 6 && !hasCoralBonus) {score += 6; hasCoralBonus = true;}
        if(object == FieldObject.L3 && !hasL3) hasL3 = true;
    }

    public Command score(FieldObject object){
        return Commands.runOnce(() -> scoreRaw(object));
    }

    public int getDeltaCoral(){
        CoralRPAlert.set((10-Corals) <= 0 && hasL3);
        AlgaeAlert.set(hasAlgaeBonus);
        CoralAlert.set(hasCoralBonus);
        return 10-Corals > 0 ? 10-Corals : 0;
    }

    public Command descore(){
        return Commands.runOnce(() -> {
            Algaes --;
            score -= 5;
        });
    }

    public void withDefault(double L1, double L2, double L3){
        for(int i = 0; i < L1; i++) scoreRaw(FieldObject.L1);
        for(int i = 0; i < L2; i++) scoreRaw(FieldObject.L2);
        for(int i = 0; i < L3; i++) scoreRaw(FieldObject.L3);
    }
    

    public static Scoring getInstance(){
        if(scoring == null) scoring = new Scoring();
        return scoring;
    }
}
