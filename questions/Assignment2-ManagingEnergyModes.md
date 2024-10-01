Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**Note: All current measurements shall be taken at a 2.25 second period with an LED on-time of 175ms. All average currents should be taken at a time scale of 200mS/div. See the pop-up menu in the lower-right corner of the Energy Profiler window**

**1. Fill in the below chart based on currents measured in each energy mode, replacing each TBD with measured values.  Use the [Selecting Ranges](https://www.silabs.com/documents/public/user-guides/ug343-multinode-energy-profiler.pdf) feature of the profiler to select the appropriate measurement range.  Your measurements should be accurate to 10%**

Energy Mode | Period (ms) | LED On Time (ms) |Period average current (uA) | Average Current with LED off (uA) | Average Current with LED On (uA)
------------| ------------|------------------|----------------------------|-----------------------------------|---------------------------------
EM0         |    2250      |       175        |          4890               |           4860                     |         5350
EM1         |    2250      |       175        |          3470               |           3440                     |         	3920
EM2         |    2250      |       174        |          43.37               |           2.84                     |         511.93
EM3         |    2210      |       174        |          42.80               |           2.85                     |         506.88



**2. ScreenShots**  

***EM0***  
Period average current    
![em0_avg_current_period][em0_avg_current_period]  
Average Current with LED ***off***  
![em0_avg_current_ledoff][em0_avg_current_ledoff]  
Average Current with LED ***on***  
![em0_avg_current_ledon][em0_avg_current_ledon]  

***EM1***  
Period average current    
![em1_avg_current_period][em1_avg_current_period]  
Average Current with LED ***off***  
![em1_avg_current_ledoff][em1_avg_current_ledoff]  
Average Current with LED ***on***  
![em1_avg_current_ledon][em1_avg_current_ledon]  

***EM2***  
Period average current  
![em2_avg_current_period][em2_avg_current_period]  
Average Current with LED ***off***  
![em2_avg_current_ledoff][em2_avg_current_ledoff]  
Average Current with LED ***on***  
![em2_avg_current_ledon][em2_avg_current_ledon]   
LED measurement - Period   
![em2_led_period][em2_led_period]  
LED measurement - LED on time   
![em2_led_ledOnTime][em2_led_ledOnTime]  

***EM3***  
Period average current    
![em3_avg_current_period][em3_avg_current_period]  
Average Current with LED ***off***  
![em3_avg_current_ledoff][em3_avg_current_ledoff]   
Average Current with LED ***on***  
![em3_avg_current_ledon][em3_avg_current_ledon]   
LED measurement - Period   
![em3_led_period][em3_led_period]  
LED measurement - LED on time   
![em3_led_ledOnTime][em3_led_ledOnTime]  

[em0_avg_current_period]: screenshots/em0_avg_current_period.JPG "em0_avg_current_period"
[em0_avg_current_ledoff]: screenshots/em0_avg_current_ledoff.JPG "em0_avg_current_ledoff"
[em0_avg_current_ledon]: screenshots/em0_avg_current_ledon.JPG "em0_avg_current_ledon"

[em1_avg_current_period]: screenshots/em1_avg_current_period.JPG "em1_avg_current_period"
[em1_avg_current_ledoff]: screenshots/em1_avg_current_ledoff.JPG "em1_avg_current_ledoff"
[em1_avg_current_ledon]: screenshots/em1_avg_current_ledon.JPG "em1_avg_current_ledon"

[em2_avg_current_period]: screenshots/em2_avg_current_period.JPG "em2_avg_current_period"
[em2_avg_current_ledoff]: screenshots/em2_avg_current_ledoff.JPG "em2_avg_current_ledoff"
[em2_avg_current_ledon]: screenshots/em2_avg_current_ledon.JPG "em2_avg_current_ledon"
[em2_led_period]: screenshots/em2_led_period.JPG "em2_led_period"
[em2_led_ledOnTime]: screenshots/em2_led_ledOnTime.JPG "em2_led_ledOnTime"

[em3_avg_current_period]: screenshots/em3_avg_current_period.JPG "em3_avg_current_period"
[em3_avg_current_ledoff]: screenshots/em3_avg_current_ledoff.JPG "em3_avg_current_ledoff"
[em3_avg_current_ledon]: screenshots/em3_avg_current_ledon.JPG "em3_avg_current_ledon"
[em3_led_period]: screenshots/em3_led_period.JPG "em3_led_period"
[em3_led_ledOnTime]: screenshots/em3_led_ledOnTime.JPG "em3_led_ledOnTime"
