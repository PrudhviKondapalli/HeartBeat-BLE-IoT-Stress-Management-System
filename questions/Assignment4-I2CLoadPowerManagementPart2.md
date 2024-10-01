Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements with logging disabled to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See the file Instructions to add screenshots in assignment.docx in the ECEN 5823 Student Public Folder.* 

1. What is the average current per period? (Remember, once you have measured your average current, average current is average current over all time. Average current doesn’t carry with it the units of the timespan over which it was measured).
   Answer: 23.25 uA
   <br>Screenshot:  
   ![Avg_current_per_period](screenshots/assignment4/avg_current_per_period.JPG)  

2. What is the ave current from the time we sleep the MCU to EM3 until we power-on the 7021 in response to the LETIMER0 UF IRQ?
   Answer: 3.87 uA
   <br>Screenshot:  
   ![Avg_current_LPM_Off](screenshots/assignment4/avg_current_lpm_off.JPG)  

3. What is the ave current from the time we power-on the 7021 until we get the COMP1 IRQ indicating that the 7021's maximum time for conversion (measurement) has expired.
   Answer: 181.60 uA
   <br>Screenshot:  
   ![Avg_current_LPM_Off](screenshots/assignment4/avg_current_lpm_on.JPG)  

4. How long is the Si7021 Powered On for 1 temperature reading?
   Answer: 103.80 ms
   <br>Screenshot:  
   ![duration_lpm_on](screenshots/assignment4/on_time_Si7021.JPG)  

5. Given the average current per period from Q1, calculate the operating time of the system for a 1000mAh battery? - ignoring battery manufacturers, efficiencies and battery derating - just a first-order calculation.
   Answer (in hours): 1000 mA / 0.02325mA = 43010 hours.
   
6. How has the power consumption performance of your design changed since the previous assignment?
   Answer: The power consumption performance of the design has significantly improved (went down) from Assignment 3 due to the fact that
   we are now using an interrupt driven I2C to write and read to the Si7021. In Assignment 3, the program used a polling method 
   to implement the timer delay between the I2C transfers while in this lab, we have a interrupt driven state machine which will put the
   device in EM3 during the waits which has greatly reduced the current consumption by the MCU. The current consumption per period is now 23.25 uA
   over the 3 second period in A4, while the current consumption per period was 147.87 uA over the 3 second period in A3. 
   


