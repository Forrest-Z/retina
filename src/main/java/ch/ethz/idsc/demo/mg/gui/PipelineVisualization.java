// code by mg
package ch.ethz.idsc.demo.mg.gui;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.WindowConstants;

import ch.ethz.idsc.owl.bot.util.UserHome;

// sets up the window for all the pipelineFrames
public class PipelineVisualization {
  private JFrame jFrame = new JFrame();
  private BufferedImage[] bufferedImage = new BufferedImage[2];
  private JComponent jComponent = new JComponent() {
    @Override
    protected void paintComponent(Graphics graphics) {
      graphics.drawString("Raw event stream", 50, 13);
      graphics.drawImage(bufferedImage[0], 50, 20, null);
      graphics.drawString("Filtered event stream with tracked objects", 50, 213);
      graphics.drawImage(bufferedImage[1], 50, 220, null);
    }
  };
  private int imageCount = 0;

  public PipelineVisualization() {
    jFrame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    jFrame.setContentPane(jComponent);
    jFrame.setBounds(100, 100, 350, 450);
    jFrame.setVisible(true);
    bufferedImage[0] = new BufferedImage(1, 1, BufferedImage.TYPE_BYTE_GRAY);
    bufferedImage[1] = new BufferedImage(1, 1, BufferedImage.TYPE_BYTE_GRAY);
  }

  public void setImage(BufferedImage bufferedImage, int imgNumber) {
    this.bufferedImage[imgNumber] = bufferedImage;
    this.bufferedImage[imgNumber] = bufferedImage;
    jComponent.repaint();
  }

  public void saveImages() throws IOException {
    imageCount++;
    ImageIO.write(bufferedImage[0], "png", UserHome.Pictures(String.format("example%03d.png", imageCount)));
    ImageIO.write(bufferedImage[1], "png", UserHome.Pictures(String.format("exampleFiltered%03d.png", imageCount)));
    System.out.printf("Images saved as example%03d.png\n", imageCount);
  }
}
