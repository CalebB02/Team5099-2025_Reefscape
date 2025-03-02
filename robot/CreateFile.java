package frc.robot;
import java.io.File; 
import java.io.IOException;  
import java.io.FileWriter;

public class CreateFile {
  private static File file;
  private static FileWriter writer;

  public static void createLog() throws IOException {
    file = new File("log"+(int)(Math.random()*10000)+".txt");
    writer = new FileWriter(file);
  }

  public static void addValues(double time, double driveVoltage, double angle) throws IOException {
    writer.write(time+": \t\t"+driveVoltage+"\t"+angle+"\t\n");
    writer.close();
  }
}
