/*
 * Dual-UART FRDM-KL25Z
 *
 * UART0: PC <-> Placa   (PTA1 RX, PTA2 TX)
 * UART1: Placa <-> Placa (PTE1 RX, PTE0 TX)
 *
 * Ciclo: 5s RX -> 5s TX
 * Botão (PTA16) força o modo definido por start_rx:
 *   start_rx = true  -> força RX
 *   start_rx = false -> força TX
 *
 * Fluxos:
 *  - Mensagens vindas do PC (UART0) são guardadas em pc_msgq e só enviadas
 *    pela UART1 durante o próximo período TX.
 *  - Mensagens vindas da outra placa (UART1) são guardadas em link_msgq e
 *    só enviadas ao PC (UART0) durante o próximo período TX.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define UART_PC_NODE    DT_NODELABEL(uart0)
#define UART_LINK_NODE  DT_NODELABEL(uart1)

#define MSG_SIZE    64   /* aumentado para margem */
#define CHECK_MS    100
#define RX_TIME_MS  5000
#define TX_TIME_MS  5000

/* Filas */
K_MSGQ_DEFINE(pc_msgq,   MSG_SIZE, 16, 4); /* mensagens do PC a enviar no próximo TX */
K_MSGQ_DEFINE(link_msgq, MSG_SIZE, 16, 4); /* mensagens vindas do link (outras placas) */

/* UART devices */
static const struct device *const uart_pc   = DEVICE_DT_GET(UART_PC_NODE);
static const struct device *const uart_link = DEVICE_DT_GET(UART_LINK_NODE);

/* Buffers temporários para ISRs */
static char pc_rx_buf[MSG_SIZE];
static int  pc_rx_pos = 0;

static char link_rx_buf[MSG_SIZE];
static int  link_rx_pos = 0;

/* Botão (PTA16) */
const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define SYNC_BUTTON_PIN 16
static struct gpio_callback button_cb_data;

/* Semáforo sinalizando botão */
K_SEM_DEFINE(sync_sem, 0, 1);

/* -------------------- UART0 ISR (PC -> placa) --------------------
   Lê bytes da UART0 (PC). Quando encontra '\n' ou '\r' finaliza a
   string e coloca em pc_msgq (K_NO_WAIT para não bloquear na ISR).
   ----------------------------------------------------------------- */
void uart_pc_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_pc)) return;
    if (!uart_irq_rx_ready(uart_pc)) return;

    while (uart_fifo_read(uart_pc, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (pc_rx_pos > 0) {
                pc_rx_buf[pc_rx_pos] = '\0';
                k_msgq_put(&pc_msgq, &pc_rx_buf, K_NO_WAIT);
                pc_rx_pos = 0;
            }
        } else if (pc_rx_pos < (MSG_SIZE - 1)) {
            pc_rx_buf[pc_rx_pos++] = c;
        }
    }
}

/* -------------------- UART1 ISR (link) --------------------------
   Lê bytes da UART1 (outra placa). Quando encontra '\n' ou '\r'
   finaliza e coloca em link_msgq.
   ---------------------------------------------------------------- */
void uart_link_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_link)) return;
    if (!uart_irq_rx_ready(uart_link)) return;

    while (uart_fifo_read(uart_link, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (link_rx_pos > 0) {
                link_rx_buf[link_rx_pos] = '\0';
                k_msgq_put(&link_msgq, &link_rx_buf, K_NO_WAIT);
                link_rx_pos = 0;
            }
        } else if (link_rx_pos < (MSG_SIZE - 1)) {
            link_rx_pos++;
            link_rx_buf[link_rx_pos - 1] = c; /* store and advance */
        }
    }
}

/* -------------------- Envio para PC (UART0) -------------------- */
void pc_print(const char *s)
{
    while (*s) {
        uart_poll_out(uart_pc, *s++);
    }
}

/* -------------------- Envio para outra placa (UART1) ---------- */
void link_send(const char *s)
{
    while (*s) {
        uart_poll_out(uart_link, *s++);
    }
}

/* -------------------- Botão ISR ------------------------------- */
void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&sync_sem);
}

/* -------------------- Função principal ------------------------ */
int main(void)
{
    char msg[MSG_SIZE];

    /* Ajuste manual em cada placa: a placa que prefere RX coloque true.
     * A outra placa coloque false. */
    bool start_rx = false;

    enum { MODE_RX = 0, MODE_TX = 1 };
    int current_mode = start_rx ? MODE_RX : MODE_TX;

    /* Verificações iniciais */
    if (!device_is_ready(uart_pc)) {
        /* sem uart PC, não dá para logar — aborta para evitar comportamento indefinido */
        return 0;
    }
    if (!device_is_ready(uart_link)) {
        pc_print("UART link (uart1) não encontrada!\r\n");
        return 0;
    }

    /* Configura callbacks e habilita RX IRQs */
    uart_irq_callback_user_data_set(uart_pc, uart_pc_cb, NULL);
    uart_irq_rx_enable(uart_pc);

    uart_irq_callback_user_data_set(uart_link, uart_link_cb, NULL);
    uart_irq_rx_enable(uart_link);

    /* Configura botão PTA16 */
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

    /* Loop principal — máquina de estados */
    while (1) {

        if (current_mode == MODE_RX) {
            pc_print(">> Entrando no modo RX...\r\n");
            /* Durante RX: recebemos via UART1 (ISR) e armazenamos em link_msgq.
             * Além disso, recebemos do PC via UART0 (ISR) e guardamos em pc_msgq,
             * mas NÃO retransmitimos até o próximo TX.
             */
            int elapsed = 0;
            while (elapsed < RX_TIME_MS) {

                /* Botão: força o modo definido por start_rx */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    int forced = start_rx ? MODE_RX : MODE_TX;
                    if (forced == MODE_RX) {
                        pc_print("Botão pressionado -> Forçando RX (modo preferido).\r\n");
                    } else {
                        pc_print("Botão pressionado -> Forçando TX (modo preferido).\r\n");
                    }
                    current_mode = forced;
                    break; /* sai do período atual e aplica novo estado */
                }

                /* Opcional: mostrar no PC as mensagens vindas do link (somente para debug);
                 * no seu fluxo desejado, as mensagens recebidas via link são entregues ao
                 * PC somente na próxima TX daquele dispositivo. Se preferir ver em tempo
                 * real, descomente a parte abaixo. Atualmente deixarei comentado.
                 */
                /*
                while (k_msgq_get(&link_msgq, &msg, K_NO_WAIT) == 0) {
                    pc_print("[DEBUG RX-IMEDIATO] ");
                    pc_print(msg);
                    pc_print("\r\n");
                }
                */

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* Se o loop terminou normalmente, passa para TX */
            if (current_mode == MODE_RX) {
                current_mode = MODE_TX;
            }
        }

        else { /* MODE_TX */
            pc_print(">> Entrando no modo TX...\r\n");

            int elapsed = 0;
            while (elapsed < TX_TIME_MS) {

                /* Botão: força o modo definido por start_rx */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    int forced = start_rx ? MODE_RX : MODE_TX;
                    if (forced == MODE_RX) {
                        pc_print("Botão pressionado -> Forçando RX (modo preferido).\r\n");
                    } else {
                        pc_print("Botão pressionado -> Forçando TX (modo preferido).\r\n");
                    }
                    current_mode = forced;
                    break; /* sai do período atual e aplica novo estado */
                }

                /* 1) Enviar para o PC todas as mensagens recebidas via link (link_msgq) */
                while (k_msgq_get(&link_msgq, &msg, K_NO_WAIT) == 0) {
                    pc_print("[DELIVERED FROM LINK] ");
                    pc_print(msg);
                    pc_print("\r\n");
                }

                /* 2) Enviar para a outra placa todas as mensagens recebidas do PC (pc_msgq) */
                while (k_msgq_get(&pc_msgq, &msg, K_NO_WAIT) == 0) {
                    /* enviar pela UART1 (link) */
                    link_send(msg);
                    link_send("\r\n"); /* garante newline no envio */
                }

                /* Se não houver mensagens, podemos mandar um PING periódico (opcional) */
                /* link_send("PING\r\n"); */

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* Se ainda estiver em TX ao finalizar, passar para RX */
            if (current_mode == MODE_TX) {
                current_mode = MODE_RX;
            }
        }

        /* continua o loop com novo estado (se foi forçado, já foi setado acima) */
    }

    return 0;
}
