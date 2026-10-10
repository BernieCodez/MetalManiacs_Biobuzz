package org.firstinspires.ftc.teamcode.util;
import org.firstinspires.ftc.teamcode.Profile;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad.*;
public class Drivers {
    Drivers drivers;
    public static Profile activeDriver;

    public final static Profile Ahmed = new Profile();
    static{Ahmed.REMOVE_DRIFT = Button.B;}
    public final static Profile Sparsh = new Profile();
    static{Sparsh.REMOVE_DRIFT = Button.B;}


    public static final Profile[] DRIVERS = {
            Ahmed,
            Sparsh,



    };
}
