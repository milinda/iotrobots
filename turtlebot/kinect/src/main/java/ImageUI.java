import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;

public class ImageUI extends JFrame {
    private BufferedImage image = null;

    public ImageUI() throws HeadlessException {
        setTitle("My Empty Frame");
        setSize(1204,768); // default size is 0,0
        setLocation(10, 200); // default is 0,0 (top left corner)

        // Add Panels
        Container contentPane = getContentPane();
        contentPane.add(new TextPanel());
    }

    public static void main(String[] args) {
            JFrame f = new ImageUI();
            f.show();
    }

    class TextPanel extends JPanel {

        public void paint(Graphics g) {
            g.drawImage(image, 0, 0, null);
            repaint();
        }
    }

    public void setImage(BufferedImage image) {
        this.image = image;
    }
}
