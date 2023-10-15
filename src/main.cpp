#include <Arduino.h>

extern "C" {
  #include "can2040.h"
}
static struct can2040 cbus1, cbus2;

static void
can2040_cb1(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    // Add message processing code here...
}

static void
can2040_cb2(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    // Add message processing code here...
}


static void
PIOx_IRQHandler1(void)
{
    can2040_pio_irq_handler(&cbus1);
}

static void
PIOx_IRQHandler2(void)
{
    can2040_pio_irq_handler(&cbus2);
}

void
canbus1_setup(void)
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus1, pio_num);
    can2040_callback_config(&cbus1, can2040_cb1);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0n, PIOx_IRQHandler1);
    NVIC_SetPriority(PIO0_IRQ_0n, 1);
    NVIC_EnableIRQ(PIO0_IRQ_0n);

    // Start canbus
    can2040_start(&cbus1, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void
canbus2_setup(void)
{
    uint32_t pio_num = 1;
    uint32_t sys_clock = 125000000, bitrate = 500000;
    uint32_t gpio_rx = 6, gpio_tx = 7;

    // Setup canbus
    can2040_setup(&cbus2, pio_num);
    can2040_callback_config(&cbus2, can2040_cb2);

    // Enable irqs
    irq_set_exclusive_handler(PIO1_IRQ_0n, PIOx_IRQHandler2);
    NVIC_SetPriority(PIO1_IRQ_0n, 1);
    NVIC_EnableIRQ(PIO1_IRQ_0n);

    // Start canbus
    can2040_start(&cbus2, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void setup() {
  // put your setup code here, to run once:
  canbus1_setup();
  canbus2_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
}
