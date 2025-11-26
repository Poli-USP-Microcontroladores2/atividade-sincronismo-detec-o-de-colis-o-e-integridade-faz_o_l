/* Código final corrigido — UART0 (PC <-> Placa) e UART1 (Placa <-> Placa)
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
 * Ambas as placas têm ciclo automático de:
 *     5 s RX -> 5 s TX -> repete
 *
 * O botão força entrar no modo preferencial definido pela flag start_rx.
 * Se start_rx = true  -> força RX
 * Se start_rx = false -> força TX
 *
 * Correção: agora o código usa um estado corrente (RX ou TX). O botão
 * força o estado para o modo definido por start_rx e o ciclo continua
 * a partir desse ponto (não retorna para onde parou).
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

/* Fila para mensagens vindas da outra placa (UART1) */
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

/* Semáforo sinalizando botão */
K_SEM_DEFINE(sync_sem, 0, 1);

/* ================= UART1 ISR (entre placas) ================= */
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
        } else if (rx_pos < MSG_SIZE - 1) {
            rx_buf[rx_pos++] = c;
        }
    }
}

/* UART0 -> PC (print) */
void pc_print(const char *s)
{
    while (*s) uart_poll_out(uart_pc, *s++);
}

/* UART1 -> outra placa (envio) */
void link_send(const char *s)
{
    while (*s) uart_poll_out(uart_link, *s++);
}

/* Botão ISR */
void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&sync_sem);
}

/* ------------------------------------------------------------------ */
/* MAIN corrigido: usa máquina de estado simples (RX/TX)              */
/* ------------------------------------------------------------------ */

int main(void)
{
    char msg[MSG_SIZE];

    /* <-- ajuste manual em cada placa --> */
    bool start_rx = true;  /* se true: preferencial RX; se false: preferencial TX */

    /* estado corrente (inicia conforme start_rx) */
    enum { MODE_RX = 0, MODE_TX = 1 } ;
    int current_mode = start_rx ? MODE_RX : MODE_TX;

    /* validações iniciais */
    if (!device_is_ready(uart_pc)) {
        /* Não conseguimos imprimir no PC, mas retornamos para travar */
        return 0;
    }
    if (!device_is_ready(uart_link)) {
        pc_print("UART de link não pronta!\r\n");
        return 0;
    }

    /* ativa ISR da UART1 (link) */
    uart_irq_callback_user_data_set(uart_link, link_uart_cb, NULL);
    uart_irq_rx_enable(uart_link);

    /* configura botão */
    if (!device_is_ready(gpioa_dev)) {
        pc_print("GPIOA não pronto!\r\n");
        return 0;
    }
    gpio_pin_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, sync_button_pressed, BIT(SYNC_BUTTON_PIN));
    gpio_add_callback(gpioa_dev, &button_cb_data);

    pc_print("Sistema iniciado.\r\n");
    pc_print(start_rx ? "Modo preferencial: RX\r\n" : "Modo preferencial: TX\r\n");

    /* loop principal com máquina de estados */
    while (1) {

        if (current_mode == MODE_RX) {
            pc_print(">> Entrando no modo RX...\r\n");
            k_msgq_purge(&link_msgq); /* opcional: limpar buffer no início do RX */

            int elapsed = 0;
            while (elapsed < RX_TIME_MS) {

                /* botão pressionado: força para o modo desejado (start_rx) */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    if (start_rx) {
                        pc_print("Botão pressionado -> Forçando RX (modo preferido).\r\n");
                        current_mode = MODE_RX; /* já está em RX, mantemos */
                    } else {
                        pc_print("Botão pressionado -> Forçando TX (modo preferido).\r\n");
                        current_mode = MODE_TX; /* salta imediatamente para TX */
                    }
                    goto next_state; /* sai do período atual e aplica novo estado */
                }

                /* imprime no PC tudo que recebeu da outra placa (se houver) */
                while (k_msgq_get(&link_msgq, &msg, K_NO_WAIT) == 0) {
                    pc_print("[RX] ");
                    pc_print(msg);
                    pc_print("\r\n");
                }

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* tempo RX aí acabou normalmente; passa para TX */
            current_mode = MODE_TX;
        }

        else { /* MODE_TX */
            pc_print(">> Entrando no modo TX...\r\n");

            int elapsed = 0;
            while (elapsed < TX_TIME_MS) {

                /* botão pressionado: força para o modo desejado (start_rx) */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    if (start_rx) {
                        pc_print("Botão pressionado -> Forçando RX (modo preferido).\r_print");
                        /* se preferir RX, muda agora para RX */
                        current_mode = MODE_RX;
                    } else {
                        pc_print("Botão pressionado -> Forçando TX (modo preferido).\r\n");
                        current_mode = MODE_TX; /* já está em TX, mantemos */
                    }
                    goto next_state; /* sai do período atual e aplica novo estado */
                }

                /* envio periódico ao link (ou esvaziar fila PC->link se implementar) */
                link_send("PING\r\n");

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* tempo TX acabou normalmente; passa para RX */
            current_mode = MODE_RX;
        }

    next_state:
        /* continue imediatamente com o novo estado no loop (sem voltar ao ponto anterior) */
        continue;
    }

    /* nunca alcança */
    return 0;
}
