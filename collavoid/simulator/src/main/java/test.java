import simbad.gui.Simbad;

public class test {
    public static void main(String[] args) {
        // request antialising
        System.setProperty("j3d.implicitAntialiasing", "true");
        // create Simbad instance with given environment
        Simbad frame = new Simbad(new BumpersDemo(), false);


    }
}
