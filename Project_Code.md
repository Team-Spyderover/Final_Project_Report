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
3. The following code implements the same functionality but removes the gesture control and let user control the car using the keyboardof their machine. The ```W``` key moves the car forward, ```s``` backwards and ```F``` to stop.
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
### Code for generation of toggle signal to control QTPY from PICO4ML

We adapted the code of ```mainfunctions.cpp``` in Pi inbuild person detection libraries to generate a toggle signal on pin 21 of pico4ml as soon as the score fo person face reaches 75 percent.

```
#include "main_functions.h"
#include <LCD_st7735.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>

#include "detection_responder.h"
#include "image_provider.h"
#include "model_settings.h"
#include "person_detect_model_data.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

const uint LED_PIN = 25;
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define RESPONSE_PIN 21

// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter    *error_reporter = nullptr;
const tflite::Model      *model          = nullptr;
tflite::MicroInterpreter *interpreter    = nullptr;
TfLiteTensor             *input          = nullptr;

// In order to use optimized tensorflow lite kernels, a signed int8_t quantized
// model is preferred over the legacy unsigned model format. This means that
// throughout this project, input images must be converted from unisgned to
// signed format. The easiest and quickest way to convert from unsigned to
// signed 8-bit integers is to subtract 128 from the unsigned value to get a
// signed value.

// An area of memory to use for input, output, and intermediate arrays.
constexpr int  kTensorArenaSize = 54 * 1024 + 27 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

#ifndef DO_NOT_OUTPUT_TO_UART

// RX interrupt handler
void on_uart_rx() {
  char cameraCommand = 0;
  while (uart_is_readable(UART_ID)) {
    cameraCommand = uart_getc(UART_ID);
    // Can we send it back?
    if (uart_is_writable(UART_ID)) {
      uart_putc(UART_ID, cameraCommand);
    }
  }
}

void setup_uart() {
  // Set up our UART with the required speed.
  uint baud = uart_init(UART_ID, BAUD_RATE);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(UART_ID, false);
  // Set up a RX interrupt
  // We need to set up the handler first
  // Select correct interrupt for the UART we are using
  int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(UART_ID, true, false);
}

#else
void setup_uart() {}
#endif

// The name of this function is important for Arduino compatibility.
void setup() {
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_put(LED_PIN, !gpio_get(LED_PIN));

  gpio_init(RESPONSE_PIN);
  gpio_set_dir(RESPONSE_PIN, GPIO_OUT);

  setup_uart();
  stdio_usb_init();
  TfLiteStatus setup_status = ScreenInit(error_reporter);
  if (setup_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Set up failed\n");
  }
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
#if 1
  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_person_detect_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  //
  // tflite::AllOpsResolver resolver;
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroMutableOpResolver<5> micro_op_resolver;
  micro_op_resolver.AddAveragePool2D();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddDepthwiseConv2D();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddSoftmax();

  // Build an interpreter to run the model with.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroInterpreter static_interpreter(
    model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);

#endif

  gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

// The name of this function is important for Arduino compatibility.
void loop() {
  gpio_put(LED_PIN, !gpio_get(LED_PIN));
#if EXECUTION_TIME
  TF_LITE_MICRO_EXECUTION_TIME_BEGIN
  TF_LITE_MICRO_EXECUTION_TIME_SNIPPET_START(error_reporter)
#endif
  // Get image from provider.
  if (kTfLiteOk
      != GetImage(error_reporter, kNumCols, kNumRows, kNumChannels, input->data.int8)) {
    TF_LITE_REPORT_ERROR(error_reporter, "Image capture failed.");
  }

#if EXECUTION_TIME
  TF_LITE_MICRO_EXECUTION_TIME_SNIPPET_END(error_reporter, "GetImage")
  TF_LITE_MICRO_EXECUTION_TIME_SNIPPET_START(error_reporter)
#endif
  // Run the model on this input and make sure it succeeds.
  if (kTfLiteOk != interpreter->Invoke()) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed.");
  }
#if EXECUTION_TIME
  TF_LITE_MICRO_EXECUTION_TIME_SNIPPET_END(error_reporter, "Invoke")
#endif
  TfLiteTensor *output = interpreter->output(0);

  // Process the inference results.
  int8_t person_score    = output->data.uint8[kPersonIndex];
  int8_t no_person_score = output->data.uint8[kNotAPersonIndex];
  RespondToDetection(error_reporter, person_score, no_person_score);

  
  printf("in pico4ml\n");
  if(person_score > 75){
    gpio_put(RESPONSE_PIN, 1);
  } else{
    gpio_put(RESPONSE_PIN, 0);
  }

#if SCREEN
  char array[10];
  sprintf(array, "%d%%", ((person_score + 128) * 100) >> 8);
  ST7735_FillRectangle(10, 120, ST7735_WIDTH, 40, ST7735_BLACK);
  ST7735_WriteString(13, 120, array, Font_16x26, ST7735_GREEN, ST7735_BLACK);
#endif
  TF_LITE_REPORT_ERROR(error_reporter, "**********");
}
```

### Offloading to PIO

The incoming_signal toggle coming from PICO4ML board is a blocking statement for the functionality of our RP2040 QtPy which controls the RC circuitry of the remote control. Thus, to free up the processor, this control signal is fetched through PIO instead of the normal GPIO and directly used in the program.

1. The in() function in pio_assembly fetches data from the specifeid pin in the main program and pusshes the data into the ISR 32 bits at time. Since, we are working in boolean input, we populate the entire register with same value.
2. Although the value of control is being used one, the data is being read continously incase user wants an active person detection ON/OFF functionality, meaning , the remote is on only for the time a face is bein detected.
3. There is also the C - helper function, which links our .pio file to the main C code.
```

.program input
set pins, 0  ; Setting the pin direction to input

loop:
  in pins, 32 ; The input is being read continously inside the loop through GPIO into the ISR 32 bits at a time
  jmp loop


% c-sdk {
static inline void input_program_init(PIO pio, uint sm, uint offset, uint pin) {
   pio_gpio_init(pio, pin);
   pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
   pio_sm_config c = input_program_get_default_config(offset);
   sm_config_set_sideset_pins(&c, pin);
   pio_sm_init(pio, sm, offset, &c);
}
%}
```



