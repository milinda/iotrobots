import java.io.File;
import java.util.Date;
import jxl.*;
import jxl.write.*;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.FileNotFoundException;

class xlsWriter {
    public static void main(String[] args) {
	try {
	    WritableWorkbook workbook = Workbook.createWorkbook( new File("SnappyOutput.xls"));
	    
	    WritableSheet sheet = workbook.createSheet("First Sheet", 0);
	    
	    try {
		Label label = new Label(0,2, "Time");
		sheet.addCell(label);
		Label label1 = new Label(1,2, "Latency");
		sheet.addCell(label1);
		
		
		try {
		    BufferedReader reader = new BufferedReader(new FileReader("/home/leif/Desktop/LatencyTest.txt"));
		    
		    String line = null;
		    int x = 1 ;
		    double num;
		    try {
			while ((line = reader.readLine()) != null) {
			    String[] parts = line.split(" ");
			    
				num = x;
				jxl.write.Number number = new jxl.write.Number(0,x, num);
				try {
				    sheet.addCell(number);
				} catch(WriteException e) {
				    System.exit(1);
				}
				num = Double.parseDouble(parts[1]);
				jxl.write.Number number1 = new jxl.write.Number(1,x, num);
				try {
				    sheet.addCell(number1);
				} catch(WriteException e) {
				    System.exit(1);
				}
			    x++;
			}
		    } catch(IOException e) {
			System.exit(1);
		    }
		} catch(FileNotFoundException e) {
		    System.exit(1);
		}
		workbook.write(); 
		workbook.close();
	    } catch(WriteException e) {
		System.exit(1);
	    }
	} catch(IOException e) {
	    System.exit(1);
	}
    }
}
