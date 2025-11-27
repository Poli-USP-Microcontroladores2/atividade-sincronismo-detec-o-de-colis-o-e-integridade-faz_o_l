#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

/* Define os nós dos dispositivos UART */
#define UART0_NODE DT_CHOSEN(zephyr_console) // Geralmente a UART de console
#define UART1_NODE DT_NODELABEL(uart1)      // UART de comunicação inter-placa

/* Tamanho do buffer para leitura de dados em cada interrupção */
#define RX_BUF_SIZE 64

/**
 * @brief Função de callback para interrupções UART.
 *
 * Esta função é chamada sempre que ocorre um evento em UART0 ou UART1.
 * Ela lida com a retransmissão bidirecional.
 */
static void uart_bridge_callback(const struct device *dev, void *user_data)
{
    uint8_t buffer[RX_BUF_SIZE];
    int len;

    // Obtém os ponteiros para os dispositivos UART
    const struct device *uart0 = DEVICE_DT_GET(UART0_NODE);
    const struct device *uart1 = DEVICE_DT_GET(UART1_NODE);
    const struct device *dest_dev = NULL;

    // 1. Determina a UART de destino (o oposto da UART que gerou a interrupção)
    if (dev == uart0) {
        // Interrupção veio da UART0 (PC). O destino é a UART1 (Outra Placa).
        dest_dev = uart1;
    } else if (dev == uart1) {
        // Interrupção veio da UART1 (Outra Placa). O destino é a UART0 (PC/Console).
        dest_dev = uart0;
    } else {
        return; // Dispositivo desconhecido, ignora.
    }

    // 2. Verifica se a interrupção é de "Dados Prontos para Receber"
    if (uart_irq_rx_ready(dev)) {
        // Lê os dados da FIFO da UART de origem
        len = uart_fifo_read(dev, buffer, sizeof(buffer));

        if (len > 0) {
            // Retransmite os dados lidos para a UART de destino
            uart_fifo_fill(dest_dev, buffer, len);
        }
    }

    // 3. (Opcional) Verifica se a interrupção é de "FIFO de Transmissão Vazia"
    // Isso é útil se estivéssemos enviando grandes blocos de dados.
    if (uart_irq_tx_ready(dev)) {
        // Não há ação de TX complexa aqui, apenas ignoramos ou limpamos a flag
    }
}


void main(void)
{
    const struct device *uart0 = DEVICE_DT_GET(UART0_NODE);
    const struct device *uart1 = DEVICE_DT_GET(UART1_NODE);

    /* --- Checagem de prontidão --- */
    if (!device_is_ready(uart0)) {
        printk("Erro: UART0 não está pronta!\n");
        return;
    } else {
        printk("SUCESSO: UART0 (Console) pronta.\n");
    }

    if (!device_is_ready(uart1)) {
        printk("Erro: UART1 não está pronta! (Verifique Device Tree)\n");
        return;
    } else {
        printk("SUCESSO: UART1 (Inter-Placa) pronta.\n");
    }

    printk("Registrando callbacks...\n");
    
    /* --- Configuração da UART0 --- */
    uart_irq_callback_set(uart0, uart_bridge_callback);
    uart_irq_rx_enable(uart0);
    printk("UART0 RX habilitada.\n");

    /* --- Configuração da UART1 --- */
    uart_irq_callback_set(uart1, uart_bridge_callback);
    uart_irq_rx_enable(uart1);
    printk("UART1 RX habilitada.\n");

    // ... (restante do código)
}