#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

/*
 * Define os nós dos dispositivos UART.
 * A UART0 é geralmente a porta do console.
 * NOTE: DT_CHOSEN(zephyr_console) é o método recomendado para o console.
 */
#define UART0_NODE DT_CHOSEN(zephyr_console)
#define UART1_NODE DT_NODELABEL(uart1)

void main(void)
{
    /* Obtém as instâncias dos dispositivos UART */
    const struct device *uart0 = DEVICE_DT_GET(UART0_NODE);
    const struct device *uart1 = DEVICE_DT_GET(UART1_NODE);

    /* --- Checagem de prontidão --- */
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

    /* --- Loop Principal de Retransmissão Bidirecional --- */
    while (1) {
        uint8_t c;

        /*
         * PARTE 1: Retransmite dados da UART0 (PC) para a UART1 (Outra Placa)
         */
        // Tenta ler um caractere da UART0
        if (uart_poll_in(uart0, &c) == 0) {
            // Se ler com sucesso (código de retorno 0), envia para a UART1
            uart_poll_out(uart1, c);
        }

        /*
         * PARTE 2: Retransmite dados da UART1 (Outra Placa) para a UART0 (PC/Console)
         */
        // Tenta ler um caractere da UART1
        if (uart_poll_in(uart1, &c) == 0) {
            // Se ler com sucesso, envia para a UART0 (que é o console printk)
            uart_poll_out(uart0, c);
        }

        /*
         * Usa k_yield() para permitir que outras threads sejam executadas.
         * Embora k_sleep(K_MSEC(1)) funcione, k_yield() é preferível em
         * loops de polling de alta frequência para evitar o desperdício de tempo
         * de CPU enquanto espera por E/S lenta.
         */
        k_yield();
    }
}