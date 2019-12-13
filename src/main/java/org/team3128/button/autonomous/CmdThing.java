
package org.team3128.button.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.team3128.common.drive.SRXTankDrive;
import org.team3128.common.util.units.Length;

//start typing the stuff to make this a robot that isn't non-functional and bad and blank and boring and stuff thanks lol
        // - Mason Holst, "Helpful Reminders", published November 2019
    
        public class CmdThing extends CommandGroup{
            public double p = 0.1;
            public CmdThing(){
                SRXTankDrive drive = SRXTankDrive.getInstance();
                addSequential(drive.new CmdDriveStraight(86.5*Length.in, p, 10000));
                }
        }