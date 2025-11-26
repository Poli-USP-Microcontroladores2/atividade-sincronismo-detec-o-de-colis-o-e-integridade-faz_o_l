/* Código final — UART0 (PC <-> Placa) e UART1 (Placa <-> Placa)
 *
 * FRDM-KL25Z:
 *   UART0 → PC
 *     RX = PTA1
 *     TX = PTA2
 *
 *   UART1 → Comunicação entre placas
 *     RX = PTE1
 *     TX = PTE0
 *
 * Ambas as placas têm ciclo automático:
 *     5 s RX → 5 s TX → repete
 *
 * O botão interrompe o ciclo atual e força entrar no modo definido por start_rx.
 * Após forçar, o ciclo recomeça a partir desse modo escolhido.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define UART_PC_NODE   DT_NODELABEL(uart0)
#define UART_LINK_NODE DT_NODELABEL(uart1)

#define MSG_SIZE    32
#define CHECK_MS    100
#define RX_TIME_MS  5000
#define TX_TIME_MS  5000

/* Fila para mensagens vindas da outra placa */
K_MSGQ_DEFINE(link_msgq, MSG_SIZE, 10, 4);

/* UART0 = PC */
static const struct device *const uart_pc   = DEVICE_DT_GET(UART_PC_NODE);

/* UART1 = comunicação entre placas */
static const struct device *const uart_link = DEVICE_DT_GET(UART_LINK_NODE);

static char rx_buf[MSG_SIZE];
static int rx_pos = 0;

/* Botão (PTA16) */
const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define SYNC_BUTTON_PIN 16
static struct gpio_callback button_cb_data;

K_SEM_DEFINE(sync_sem, 0, 1);

/* UART1 ISR */
void link_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_link)) return;
    if (!uart_irq_rx_ready(uart_link)) return;

    while (uart_fifo_read(uart_link, &c, 1) == 1) {

        if (c == '\n' || c == '\r') {
            if (rx_pos > 0) {
                rx_buf[rx_pos] = '\0';
                k_msgq_put(&link_msgq, &rx_buf, K_NO_WAIT);
                rx_pos = 0;
            }
        } 
        else if (rx_pos < MSG_SIZE - 1) {
            rx_buf[rx_pos++] = c;
        }
    }
}

/* UART0 → PC */
void pc_print(const char *s)
{
    while (*s) uart_poll_out(uart_pc, *s++);
}

/* UART1 → placa remota */
void link_send(const char *s)
{
    while (*s) uart_poll_out(uart_link, *s++);
}

/* Botão ISR */
void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&sync_sem);
}

/* ============================================================ */
/* ========================== MAIN ============================ */
/* ============================================================ */

int main(void)
{
    char msg[MSG_SIZE];

    /*
     * true  → a placa prefere RX quando o botão for pressionado
     * false → prefere TX
     */
    bool start_rx = false;

    /* Inicialização UARTs */
    if (!device_is_ready(uart_pc))   return 0;
    if (!device_is_ready(uart_link)) return 0;

    uart_irq_callback_user_data_set(uart_link, link_uart_cb, NULL);
    uart_irq_rx_enable(uart_link);

    /* Botão */
    gpio_pin_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, sync_button_pressed, BIT(SYNC_BUTTON_PIN));
    gpio_add_callback(gpioa_dev, &button_cb_data);

    pc_print("Sistema iniciado.\r\n");
    pc_print(start_rx ? "Modo preferencial: RX\r\n" : "Modo preferencial: TX\r\n");

    /* Estado atual do ciclo */
    enum { MODE_RX, MODE_TX } mode = MODE_RX;

    while (1) {

        /* ====================================================== */
        /* ======================== RX ========================== */
        /* ====================================================== */
        if (mode == MODE_RX) {

            pc_print(">> Entrando no modo RX...\r\n");
            k_msgq_purge(&link_msgq);

            int elapsed = 0;
            while (elapsed < RX_TIME_MS) {

                /* Botão → força modo preferido e reinicia o ciclo */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    mode = start_rx ? MODE_RX : MODE_TX;
                    pc_print("Botão pressionado → Reiniciando ciclo em ");
                    pc_print(start_rx ? "RX\r\n" : "TX\r\n");
                    break;
                }

                while (k_msgq_get(&link_msgq, &msg, K_NO_WAIT) == 0) {
                    pc_print("[RX] "); pc_print(msg); pc_print("\r\n");
                }

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            if (elapsed >= RX_TIME_MS)
                mode = MODE_TX;
        }

        /* ====================================================== */
        /* ======================== TX ========================== */
        /* ====================================================== */
        else {

            pc_print(">> Entrando no modo TX...\r\n");

            int elapsed = 0;
            while (elapsed < TX_TIME_MS) {

                /* Botão → força modo preferido e reinicia ciclo */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    mode = start_rx ? MODE_RX : MODE_TX;
                    pc_print("Botão pressionado → Reiniciando ciclo em ");
                    pc_print(start_rx ? "RX\r\n" : "TX\r\n");
                    break;
                }

                link_send("PING\r\n");

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            if (elapsed >= TX_TIME_MS)
                mode = MODE_RX;
        }
    }

    return 0;
}
