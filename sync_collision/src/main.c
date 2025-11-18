
/*
 * UART com sincronização assíncrona por botão
 * Placa: FRDM-KL25Z
 *
 * PTA16: Botão de sincronização
 * UART: selecionada pelo Devicetree
 *
 * Ciclo RX/TX: 5s RX, 5s TX
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 32

#define RX_TX_CHECK_INTERVAL_MS 100  // Checagem do botão a cada 100ms
#define RX_DURATION_MS 5000          // 5s RX
#define TX_DURATION_MS 5000          // 5s TX

/* --- Fila de mensagens UART --- */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* --- UART --- */
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/* --- Botão de sincronização --- */
const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define SYNC_BUTTON_PIN 16
static struct gpio_callback button_cb_data;

/* --- Semáforo para sinalizar botão --- */
K_SEM_DEFINE(sync_sem, 0, 1);

/* --- Callback do botão --- */
void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&sync_sem);
}

/* --- ISR UART --- */
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) return;
    if (!uart_irq_rx_ready(uart_dev)) return;

    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (rx_buf_pos > 0) {
                rx_buf[rx_buf_pos] = '\0';
                k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
                rx_buf_pos = 0;
            }
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
    }
}

/* --- Função para enviar strings via UART --- */
void print_uart(char *buf)
{
    int msg_len = strlen(buf);
    for (int i = 0; i < msg_len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

/* --- Função principal --- */
int main(void)
{
    char tx_buf[MSG_SIZE];
    bool start_rx = false; // Configure manualmente a outra placa como false

    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!\n");
        return 0;
    }

    /* Configura ISR UART */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    if (ret < 0) {
        printk("Erro ao configurar callback UART: %d\n", ret);
        return 0;
    }
    uart_irq_rx_enable(uart_dev);

    /* Configura botão de sincronização */
    if (!device_is_ready(gpioa_dev)) {
        printk("GPIOA device not ready\n");
        return 0;
    }
    ret = gpio_pin_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) { printk("Erro ao configurar botão\n"); return 0; }

    ret = gpio_pin_interrupt_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) { printk("Erro ao configurar interrupção do botão\n"); return 0; }

    gpio_init_callback(&button_cb_data, sync_button_pressed, BIT(SYNC_BUTTON_PIN));
    gpio_add_callback(gpioa_dev, &button_cb_data);

    /* --- Mensagem inicial --- */
    print_uart("UART iniciado. Pressione o botão a qualquer momento para sincronizar.\r\n");

    /* --- Loop principal RX/TX --- */
    while (1) {
        if (start_rx) {
            /* --- MODO RX --- */
            k_msgq_purge(&uart_msgq);
            print_uart("Modo RX: acumulando mensagens por 5s...\r\n");

            int elapsed = 0;
            while (elapsed < RX_DURATION_MS) {
                k_sleep(K_MSEC(RX_TX_CHECK_INTERVAL_MS));
                elapsed += RX_TX_CHECK_INTERVAL_MS;

                // Verifica botão a qualquer momento
                if (k_sem_count_get(&sync_sem) > 0) {
                    k_sem_take(&sync_sem, K_NO_WAIT);
                    print_uart("Botão pressionado! Forçando sincronização...\r\n");
                    start_rx = true; // ou false na outra placa
                    break;           // sai do RX imediatamente
                }
            }

            /* --- MODO TX --- */
            print_uart("Modo TX: esvaziando fila...\r\n");

            elapsed = 0;
            while (elapsed < TX_DURATION_MS) {
                // Esvazia fila continuamente
                while (k_msgq_get(&uart_msgq, &tx_buf, K_NO_WAIT) == 0) {
                    print_uart("Eco: ");
                    print_uart(tx_buf);
                    print_uart("\r\n");
                }

                k_sleep(K_MSEC(RX_TX_CHECK_INTERVAL_MS));
                elapsed += RX_TX_CHECK_INTERVAL_MS;

                // Verifica botão
                if (k_sem_count_get(&sync_sem) > 0) {
                    k_sem_take(&sync_sem, K_NO_WAIT);
                    print_uart("Botão pressionado! Forçando sincronização...\r\n");
                    start_rx = true; // ou false na outra placa
                    break;           // sai do TX imediatamente
                }
            }

        } else {
            /* --- MODO TX inicial para sincronização --- */
            print_uart("Modo TX inicial: aguardando 5s...\r\n");

            int elapsed = 0;
            while (elapsed < TX_DURATION_MS) {
                k_sleep(K_MSEC(RX_TX_CHECK_INTERVAL_MS));
                elapsed += RX_TX_CHECK_INTERVAL_MS;

                if (k_sem_count_get(&sync_sem) > 0) {
                    k_sem_take(&sync_sem, K_NO_WAIT);
                    print_uart("Botão pressionado durante TX inicial! Forçando sincronização...\r\n");
                    start_rx = true; // alterna para RX/TX normal
                    break;
                }
            }

            start_rx = true; // depois do primeiro TX inicial, entra no ciclo normal
        }
    }

    return 0;
}
