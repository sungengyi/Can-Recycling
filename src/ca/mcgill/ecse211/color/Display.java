package ca.mcgill.ecse211.color;

import ca.mcgill.ecse211.odometer.*;
import java.text.DecimalFormat;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

  private Odometer odo;
  private TextLCD lcd;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    this.lcd = lcd;
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  public int onScreenSelection(String str) {
		int buttonChoice;
		lcd.clear();
		lcd.drawString("Press any button to", 0, 1);
		lcd.drawString("select " + str + " value.", 0, 2);
		do{
			buttonChoice= Button.waitForAnyPress();
		}while (buttonChoice != Button.ID_LEFT && 
				buttonChoice != Button.ID_RIGHT && 
				buttonChoice != Button.ID_UP &&
				buttonChoice != Button.ID_DOWN);//Block codes below until this linw is executed
		lcd.clear();
		return numberSelection(0);
	}
	/**
	 * Processing number selection of LLx, LLy, etc.
	 * @param page
	 * @return
	 */
	public int numberSelection(int page) {
		int value = -1;
		int buttonChoice;

		displayPages(page);
		do {
			buttonChoice = Button.waitForAnyPress();
		}while (buttonChoice != Button.ID_LEFT && 
				buttonChoice != Button.ID_RIGHT && 
				buttonChoice != Button.ID_ESCAPE &&
				buttonChoice != Button.ID_UP &&
				buttonChoice != Button.ID_DOWN);//Block codes below until this linw is executed

		if(buttonChoice == Button.ID_LEFT) {
			value = ( 2 * page + 1 ) % 10;
			return value;
		}else if(buttonChoice == Button.ID_RIGHT) {
			value = ( 2 * page + 2 ) % 10;
			return value;
		}else if(buttonChoice == Button.ID_UP) {
			page = ( 5 + page - 1 ) % 5;
			return numberSelection(page);
		}else if(buttonChoice == Button.ID_DOWN) {
			page = ( page + 1 ) % 5;
			return numberSelection(page);
		}else {
			return -1;
		}


	}
	/**
	 * Display number choices onto lcd screen.
	 * @param page
	 */
	public void displayPages(int page) {
		switch (page) {
		case 0:
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("   1   |   2    ", 0, 1);
			lcd.drawString("^  Up  |  Down v", 0, 2);
			lcd.drawString("Previo-|  Next  ", 0, 3);
			lcd.drawString("us Page|  Page  ", 0, 4);
			break;
		case 1:
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("   3   |   4    ", 0, 1);
			lcd.drawString("^  Up  |  Down v", 0, 2);
			lcd.drawString("Previo-|  Next  ", 0, 3);
			lcd.drawString("us Page|  Page  ", 0, 4);
			break;
		case 2:
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("   5   |   6    ", 0, 1);
			lcd.drawString("^  Up  |  Down v", 0, 2);
			lcd.drawString("Previo-|  Next  ", 0, 3);
			lcd.drawString("us Page|  Page  ", 0, 4);
			break;
		case 3:
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("   7   |   8    ", 0, 1);
			lcd.drawString("^  Up  |  Down v", 0, 2);
			lcd.drawString("Previo-|  Next  ", 0, 3);
			lcd.drawString("us Page|  Page  ", 0, 4);
			break;
		case 4:
			lcd.clear();
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("   9   |   0    ", 0, 1);
			lcd.drawString("^  Up  |  Down v", 0, 2);
			lcd.drawString("Previo-|  Next  ", 0, 3);
			lcd.drawString("us Page|  Page  ", 0, 4);
			break;
		default:
			lcd.clear();
			lcd.drawString("     ERROR!     ", 0, 0);
			lcd.drawString("  PRESS ESCAPE  ", 0, 1);
			lcd.drawString(" BUTTON TO EXIT!", 0, 2);
			break;
		}


	}
  public void run() {
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 4);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 5);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 6);


      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}

