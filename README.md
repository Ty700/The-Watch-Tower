The Watch Tower is a project that utilizes a PIC16F1829 microcontroller to measure and display various environmental parameters. The project's purpose is to create a device that can monitor a room's temperature, ambient light, and proximity of objects in the room. The collected data is then displayed on a 2x16 LCD display using the built-in I2C interface on the PIC16F1829.

To measure the surrounding room temperature, the Watch Tower uses a TMP36 temperature sensor. These sensors output a voltage that is proportional to the temperature, which can be measured using the ADCs on the PIC16F1829. The temperature is then displayed on the LCD screen.

To measure the amount of light in the area, the Watch Tower uses a photoresistor. These sensors vary their resistance or output voltage based on the ambient light level. The PIC16F1829 can again use its ADCs to measure the output voltage or resistance of the sensor and display the light level on the LCD screen. 

To determine how far an object is from the tower, it utilizes an HC-SR04 ultrasonic sensor. This sensor emits high-frequency sound waves and measures the time it takes for the sound waves to bounce back from an object. The PIC16F1829 can measure the duration of the sound wave using its built-in timers, which can be converted into distance. The distance measurement is then displayed on the LCD screen.

Finally, the Watch Tower uses a buzzer to sound an alarm if an object is closer than 15cm away from the ultrasonic sensor. The PIC16F1829 can control the buzzer using its digital output pins.
