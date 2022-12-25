# Final Project Code

### System Details
- Processor - AMD Ryzen 5 4600 H
- RAM - 16 GB
- Operating System : Windows 11 Version 22H2

### Dev Tools : 
Developer Visual Studio Code

### References : 
1) Getting Started with PICO 
2) C/C++ SDK Development

### Team Members : 
1) Prateek Bashista
2) Joyendra Roy Biswas

The project utilises 2 microntroller boards to realise the face detection and RC control functionality. The RP2040 QTPY board controllers the rc remote through analog voltages. The RP2040 PICO4ML board is primarily used to implement face detection functionality and generate the control signal to toggle QTPY.


### Code Breakdown for Control of RC circuitry and integration with IMU for gesture control

1. The following ```#define``` statements declare the various I/O of the RP2040 that control the rc circuit of the remote control. The ```OUTER_FWD_PIN 24```, ```OUTER_BCK_PIN 4```, ```INNER_FWD_PIN 6```, ```INNER_BCK_PIN 25``` are used to declare the GPIO that will generate the analog signal to mimic the button of the RC circuit. THey generate 3.3V and -3.3V to implement forward and backward motion on the car. The name of the pin indicate the particular wheel it is controlling. The 
```X_AXIS_PIN 29```, ```Y_AXIS_PIN 28``` , ```Z_AXIS_PIN 27``` take values of X,Y and Z axes from the IMU. The ```CONTROL_SIGNAL 20``` gets the toggle input from PICO4ML.

``` 
#define PICO_DEFAULT_WS2812_POWER_PIN 11

// Decaration of RC circuit control pins
#define OUTER_FWD_PIN 24
#define OUTER_BCK_PIN 4
#define INNER_FWD_PIN 6
#define INNER_BCK_PIN 25

// Declaration of IMU sensor input
#define X_AXIS_PIN 29 
#define Y_AXIS_PIN 28
#define Z_AXIS_PIN 27

// Declaration of Toggle signal from PICO4ML as input
#define CONTROL_SIGNAL 20

```
2. The following snippet show the working of the algorithm that enables functionality. First, 3 variales ```x_pos```,```y_pos``` and ```z_pos``` are used to get the values of X,Y and Z axes from the IMU. The ```x```, ```y``` and ```z``` variables act as the relatie position or  previous value. Since, no sensor can be mounted on the car, the relative position of the remote is used and its movement is used to decide left , right ,forward or backward functionality. The basic idea is that if the person is detected , the remote control turns on, and controls the car based on relative position. The current position is stored, and new values are compared to older position. With this , the direction of movement of remote is calculated.
```
    float x_pos = 0;
    float y_pos = 0;
    float z_pos = 0;
    float x;
    float y;
    float z;

    while (1) {
        if(gpio_get(incoming_signal)){
            x = x_pos;
            y = y_pos;
            z = z_pos;

            adc_select_input(3);
            uint adc_x_raw = adc_read();
            adc_select_input(2);
            uint adc_y_raw = adc_read();
            adc_select_input(1);
            uint adc_z_raw = adc_read();
            
            const float conversion_factor = (1 << 12) - 1;
            uint16_t result = adc_read();
            x_pos = adc_x_raw / conversion_factor;
            y_pos = adc_y_raw / conversion_factor;
            z_pos = adc_z_raw / conversion_factor;
            printf("X-pos: %f, Y-pos: %f, Z-pos: %f \n", x_pos, y_pos, z_pos);
            printf("Level = %d\n", level);

            if(x - x_pos > 0)
            {
                gpio_put(OUTER_FWD,1);
                gpio_put(INNER_FWD,1);
                gpio_put(OUTER_BCK,0);
                gpio_put(INNER_BCK,0);
            }
            else if(x - x_pos < 0)
            {
                gpio_put(OUTER_FWD,0);
                gpio_put(INNER_FWD,0);
                gpio_put(OUTER_BCK,1);
                gpio_put(INNER_BCK,1);
            }
            
            if(y - y_pos > 0)
            {
                gpio_put(OUTER_FWD,1);
                gpio_put(INNER_FWD,0);
                gpio_put(OUTER_BCK,0);
                gpio_put(INNER_BCK,1);
            }
            else if(y - y_pos < 0 )
            {
                gpio_put(OUTER_FWD,0);
                gpio_put(INNER_FWD,1);
                gpio_put(OUTER_BCK,1);
                gpio_put(INNER_BCK,0);
            }
            sleep_ms(500);
        }
        else{
            gpio_put(OUTER_FWD,0);
            gpio_put(INNER_FWD,0);
            gpio_put(OUTER_BCK,0);
            gpio_put(INNER_BCK,0);
            printf("No target found! Please pan the vision! \n");
            sleep_ms(500);
        }
    }

```
3. The following code implements the same fucntionality but removes the gesture control and et user control the car using the keyboardof their machine. The ```W``` key moves the car forward, ```s``` backwards and ```F``` to stop.
```
    while (1) {

    char c = getchar_timeout_us (2000);

    if(c== 'w')
    {
    gpio_put(OUTER_FWD, 1);
    gpio_put(OUTER_BCK, 0);
    gpio_put(INNER_FWD, 0);
    gpio_put(INNER_BCK, 1);
    }
    else if(c== 's')
    {
    gpio_put(OUTER_FWD, 1);
    gpio_put(OUTER_BCK, 1);
    gpio_put(INNER_FWD, 0);
    gpio_put(INNER_BCK, 0);
    }
    else if(c== 'f')
    {
    gpio_put(OUTER_FWD, 0);
    gpio_put(OUTER_BCK, 0);
    gpio_put(INNER_FWD, 0);

    gpio_put(INNER_BCK, 0);
    }

    }
}
```


