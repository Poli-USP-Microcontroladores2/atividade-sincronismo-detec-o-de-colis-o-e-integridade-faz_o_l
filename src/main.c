#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#define UART0_NODE DT_CHOSEN(zephyr_console)
#define UART1_NODE DT_NODELABEL(uart1)

void main(void)
{
    const struct device *uart0 = DEVICE_DT_GET(UART0_NODE);
    const struct device *uart1 = DEVICE_DT_GET(UART1_NODE);
    if (!device_is_ready(uart0)) {
        printk("Erro: UART0 (Console) não está pronta!\n");
        return;
    }
    if (!device_is_ready(uart1)) {
        printk("Erro: UART1 (Inter-Placa) não está pronta! Verifique o Device Tree (.dts/.overlay).\n");
        return;
    }
    printk("Bridge UART0 <-> UART1 ativada! Qualquer dado será retransmitido.\n");
    printk("-----------------------------------------------------------------\n");
    while (1) {
        uint8_t c;
        if (uart_poll_in(uart0, &c) == 0) {
            uart_poll_out(uart1, c);
        }
        if (uart_poll_in(uart1, &c) == 0) {
            uart_poll_out(uart0, c);
        }
        k_yield();
    }
}