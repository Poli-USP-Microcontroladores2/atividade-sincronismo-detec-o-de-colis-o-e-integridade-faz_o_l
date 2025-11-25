/*
 * Ciclo automático RX/TX com botão que força entrar no modo definido pela flag start_rx
 * Placa A: start_rx = true  → botão força modo RX
 * Placa B: start_rx = false → botão força modo TX
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define MSG_SIZE 32

#define CHECK_MS 100
#define RX_MS 5000
#define TX_MS 5000

/* Fila UART */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* UART */
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static char rx_buf[MSG_SIZE];
static int rx_pos;

/* Botão */
const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define SYNC_BUTTON_PIN 16
static struct gpio_callback btn_cb;

/* Semáforo */
K_SEM_DEFINE(sync_sem, 0, 1);

/* -------------------------------------------------------------------------- */
/* INTERRUPÇÕES                                                               */
/* -------------------------------------------------------------------------- */

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&sync_sem);
}

/* UART ISR */
void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) return;
    if (!uart_irq_rx_ready(uart_dev)) return;

    while (uart_fifo_read(uart_dev, &c, 1) == 1) {

        if (c == '\n' || c == '\r') {
            if (rx_pos > 0) {
                rx_buf[rx_pos] = '\0';
                k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
                rx_pos = 0;
            }
        }
        else if (rx_pos < MSG_SIZE - 1) {
            rx_buf[rx_pos++] = c;
        }
    }
}

/* Envio UART */
void print_uart(const char *msg)
{
    while (*msg)
        uart_poll_out(uart_dev, *msg++);
}

/* -------------------------------------------------------------------------- */
/* PROGRAMA PRINCIPAL                                                         */
/* -------------------------------------------------------------------------- */

int main(void)
{
    char tx_buf[MSG_SIZE];

    /* CONFIGURE ESTA FLAG EM CADA PLACA */
    bool start_rx = true;   // PLACA A
    //bool start_rx = false; // PLACA B

    /* Inicialização UART */
    uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
    uart_irq_rx_enable(uart_dev);

    /* Botão */
    gpio_pin_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&btn_cb, button_pressed, BIT(SYNC_BUTTON_PIN));
    gpio_add_callback(gpioa_dev, &btn_cb);

    print_uart("Sistema iniciado.\r\n");

    /* ====================== LOOP PRINCIPAL ====================== */
    bool mode_rx = true;  // COMEÇA EM RX

    while (1) {

        /* -------------------------------------- */
        /* SE BOTÃO FOR PRESSIONADO → FORÇA MODO  */
        /* -------------------------------------- */
        if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
            if (start_rx) {
                print_uart("[BOTAO] Forçando entrar em RX.\r\n");
                mode_rx = true;
            } else {
                print_uart("[BOTAO] Forçando entrar em TX.\r\n");
                mode_rx = false;
            }
        }

        /* =================== MODO RX =================== */
        if (mode_rx) {
            print_uart(">> Entrando em RX...\r\n");
            k_msgq_purge(&uart_msgq);

            int elapsed = 0;
            while (elapsed < RX_MS) {

                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    print_uart("[BOTAO] Forçando RX.\r\n");
                    mode_rx = start_rx;
                    break;
                }

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* Ao fim de 5s RX → alterna para TX */
            if (elapsed >= RX_MS)
                mode_rx = false;
        }

        /* =================== MODO TX =================== */
        else {
            print_uart(">> Entrando em TX...\r\n");

            int elapsed = 0;
            while (elapsed < TX_MS) {

                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    print_uart("[BOTAO] Forçando TX.\r\n");
                    mode_rx = start_rx;
                    break;
                }

                /* Envio das mensagens */
                while (k_msgq_get(&uart_msgq, &tx_buf, K_NO_WAIT) == 0) {
                    print_uart("Eco: ");
                    print_uart(tx_buf);
                    print_uart("\r\n");
                }

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* Ao fim de 5s TX → alterna para RX */
            if (elapsed >= TX_MS)
                mode_rx = true;
        }
    }

    return 0;
}