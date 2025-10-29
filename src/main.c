
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#define DEBOUNCE_DELAY 100 //ms
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1

struct gyro_accel_data{
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float temp;
}data;

static uint32_t last_press_time_sw1 = 0;
static uint32_t last_press_time_sw2 = 0;


enum state { WAITING=1};
enum state programState = WAITING;

void confirmPos(void) {
    printf("Button 1 pressed with debounce\n");
}

void addSpace(void) {
    printf("Button 2 pressed with debounce\n");
}

static void sw_callback(uint gpio, uint32_t eventMask) {
    //get current time to compare the time between 2 interrupt with DEBOUNCE_DELAY to avoid switch bounce
    uint32_t curr_time = to_ms_since_boot(get_absolute_time()); 

    if (gpio == SW1_PIN && (eventMask & GPIO_IRQ_EDGE_FALL)) {
        //only process the interrupt if the delay since the last interrupt for the same button
         if (curr_time - last_press_time_sw1 > DEBOUNCE_DELAY) {
            last_press_time_sw1 = curr_time;
            confirmPos(); 
        }
    } 
    else if (gpio == SW2_PIN && (eventMask & GPIO_IRQ_EDGE_FALL)) {
        if (curr_time - last_press_time_sw2 > DEBOUNCE_DELAY) {
            last_press_time_sw2 = curr_time;
            addSpace(); 
        }
        
    }

}

    


static void sensor_task(void *arg){
    (void)arg;
//init and start ICM sensor
    init_ICM42670();
    ICM42670_start_with_default_values();
    ICM42670_enable_accel_gyro_ln_mode();
   
    for(;;){

        //read sensor values and write them to global structure
        ICM42670_read_sensor_data(&data.accel_x, &data.accel_y, &data.accel_z, &data.gyro_x, &data.gyro_y, &data.gyro_z, &data.temp);

    
        //printf("sensorTask\n");
        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void print_task(void *arg){
    (void)arg;
    
    while(1){
    
    printf("gyro test: %.2f, %.2f, %.2f | %.2f, %.2f, %.2f | %.2f\n", data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z);



    
    //printf("printTask\n");
    // Do not remove this
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// Exercise 4: Uncomment the following line to activate the TinyUSB library.  
// Tehtävä 4:  Poista seuraavan rivin kommentointi aktivoidaksesi TinyUSB-kirjaston. 

/*
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}*/

int main() {

    // Exercise 4: Comment the statement stdio_init_all(); 
    //             Instead, add AT THE END OF MAIN (before vTaskStartScheduler();) adequate statements to enable the TinyUSB library and the usb-serial-debug.
    //             You can see hello_dual_cdc for help
    //             In CMakeLists.txt add the cfg-dual-usbcdc
    //             In CMakeLists.txt deactivate pico_enable_stdio_usb
    // Tehtävä 4:  Kommentoi lause stdio_init_all();
    //             Sen sijaan lisää MAIN LOPPUUN (ennen vTaskStartScheduler();) tarvittavat komennot aktivoidaksesi TinyUSB-kirjaston ja usb-serial-debugin.
    //             Voit katsoa apua esimerkistä hello_dual_cdc.
    //             Lisää CMakeLists.txt-tiedostoon cfg-dual-usbcdc
    //             Poista CMakeLists.txt-tiedostosta käytöstä pico_enable_stdio_usb

    stdio_init_all();

    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    
    
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.


    //init button 
    init_sw1();
    init_sw2();

    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, &sw_callback);
    gpio_set_irq_enabled(SW2_PIN, GPIO_IRQ_EDGE_FALL, true);


    TaskHandle_t hSensorTask, hPrintTask, hUSB = NULL;

    // Exercise 4: Uncomment this xTaskCreate to create the task that enables dual USB communication.
    // Tehtävä 4: Poista tämän xTaskCreate-rivin kommentointi luodaksesi tehtävän,
    // joka mahdollistaa kaksikanavaisen USB-viestinnän.

    /*
    xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif
    */


    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                &hSensorTask);                   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }
    result = xTaskCreate(print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hPrintTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}

