Note: For all assignments and Energy Profiler measurements you’ll be taking this semester,  Peak measurements are instantaneous measurements taken at a specific point in time. In the Energy Profiler, this is accomplished by left-clicking at a location along the time axis.
Average measurements are measurements that are taken over a time-span. In the Energy Profiler, this is accomplished by left-clicking and dragging a region along the time axis.

Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   **Answer:** When the GPIO pin is set to StrongAlternateStrong, the instantaneous current draw is ~5.41 mA.


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   **Answer:** When the GPIO pin is set to WeakAlternateWeak, the instantaneous current draw is ~5.24 mA.


**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, referencing the main board schematic, WSTK-Main-BRD4001A-A01-schematic.pdf or WSTK-Main-BRD4002A-A06-schematic.pdf, and AEM Accuracy in the ug279-brd4104a-user-guide.pdf. Both of these PDF files are available in the ECEN 5823 Student Public Folder in Google drive at: https://drive.google.com/drive/folders/1ACI8sUKakgpOLzwsGZkns3CQtc7r35bB?usp=sharing . Extra credit is available for this question and depends on your answer.**
   **Answer:** No, there is not a meaningful difference in current for question 1 and 2 because the current draw would be constrained by the 2.7K ohm series resistor in both cases. It is also true
   that looking at the current consumption of a system when the LED is off is around 4.76 mA (WeakAlternateWeak), and when the LED is on, the current consumption is 5.24 mA (WeakAlternateWeak). The difference between 
   the current consumption in the two cases is the current consumed by just the LED which is around 0.48 mA. This difference is constant (~0.5 mA) for the StrongAlternateStrong case as well.
   This means the system requires 0.48 mA in order to turn on the LED. But since there are two drive strengths (1 mA for WeakAlternateWeak, 10 mA for StrongAlternateStrong), either case is more than enough to turn on the LED, meaning the system requires 0.48 mA but 
   the drive strengths can easily drive it (both drive strengths are > 0.48 mA), which is why we don't see a meaningful difference in the currents.  <br />
   **Extra Credit 1:** Additional reason as to why there isn't a meaningful 
   difference between the two current measurements is because of the AEM Accuracy of the Blue Gecko board, which is accurate within 0.1 mA, for currents over 250 uA. (Source: ug279-brd4104a-user-guide). 
   This means between the two current measurements, the error of the measurements are within (upto) 100mA, and because of this also we aren't able to see a meaningful difference between the two drive strengths. 
   Within the current measurements we're seeing, the AEM just isn't accurate enough to pick up on the small current differences. <br />
   **Extra Credit 2:** 


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: The average current for 1 complete on-off cycle for 1 LED is 5.14 mA. 


**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: The average current for 1 complete on-off cycle for 2 LEDs is 5.33 mA.


