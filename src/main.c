/*
 * Dual-UART (PC <-> Placa) e (Placa <-> Placa)
 * FRDM-KL25Z
 *
 * UART0 -> PC    (PTA1 RX, PTA2 TX)
 * UART1 -> Link  (PTE1 RX, PTE0 TX)
 *
 * Ciclo automático: 5s RX -> 5s TX -> repete
 * Botão (PTA16) força entrar no modo definido por start_rx:
 *   start_rx = true  -> forçar RX
 *   start_rx = false -> forçar TX
 *
 * Fluxo PC->Placa stored: mensagens vindas do PC (UART0) são guardadas
 * e só repassadas pela UART1 (link) durante o próximo TX.
 * Placa B entrega ao seu PC apenas se estiver em modo RX (imprime link_msgq).
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

/* Fila: mensagens recebidas da outra placa (UART1 -> link_msgq) */
K_MSGQ_DEFINE(link_msgq, MSG_SIZE, 10, 4);
/* Fila: mensagens recebidas do PC (UART0) que devem ser enviadas no próximo TX */
K_MSGQ_DEFINE(pc_msgq, MSG_SIZE, 10, 4);

/* UART devices */
static const struct device *const uart_pc   = DEVICE_DT_GET(UART_PC_NODE);
static const struct device *const uart_link = DEVICE_DT_GET(UART_LINK_NODE);

/* Buffers e posições para ISRs */
static char link_rx_buf[MSG_SIZE];
static int  link_rx_pos = 0;

static char pc_rx_buf[MSG_SIZE];
static int  pc_rx_pos = 0;

/* Botão (PTA16) */
const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define SYNC_BUTTON_PIN 16
static struct gpio_callback button_cb_data;

/* Semáforo sinalizando botão */
K_SEM_DEFINE(sync_sem, 0, 1);

/* ------------------- ISR: UART1 (link) - recebe da outra placa -------------- */
void link_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_link)) return;
    if (!uart_irq_rx_ready(uart_link)) return;

    while (uart_fifo_read(uart_link, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (link_rx_pos > 0) {
                link_rx_buf[link_rx_pos] = '\0';
                /* tenta enfileirar, descarta se cheio */
                k_msgq_put(&link_msgq, &link_rx_buf, K_NO_WAIT);
                link_rx_pos = 0;
            }
        } else if (link_rx_pos < MSG_SIZE - 1) {
            link_rx_buf[link_rx_pos++] = (char)c;
        }
    }
}

/* ------------------- ISR: UART0 (pc) - recebe do PC ------------------------- */
void pc_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_pc)) return;
    if (!uart_irq_rx_ready(uart_pc)) return;

    while (uart_fifo_read(uart_pc, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (pc_rx_pos > 0) {
                pc_rx_buf[pc_rx_pos] = '\0';
                /* Guardar mensagem vinda do PC para envio no próximo TX */
                k_msgq_put(&pc_msgq, &pc_rx_buf, K_NO_WAIT);
                pc_rx_pos = 0;
            }
        } else if (pc_rx_pos < MSG_SIZE - 1) {
            pc_rx_buf[pc_rx_pos++] = (char)c;
        }
    }
}

/* ------------------- Envio helpers ---------------------------------------- */
void pc_print(const char *s)
{
    while (*s) {
        uart_poll_out(uart_pc, *s++);
    }
}

void link_send(const char *s)
{
    while (*s) {
        uart_poll_out(uart_link, *s++);
    }
}

/* ------------------- Botão ISR ------------------------------------------- */
void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&sync_sem);
}

/* ------------------- Main ------------------------------------------------- */
int main(void)
{
    char msg[MSG_SIZE];

    /* Ajuste manual em cada placa:
     *  - placa A: start_rx = true  (prefere RX)
     *  - placa B: start_rx = false (prefere TX)
     */
    bool start_rx = false;

    enum { MODE_RX = 0, MODE_TX = 1 };
    int current_mode = start_rx ? MODE_RX : MODE_TX;

    /* inicializações */
    if (!device_is_ready(uart_pc)) {
        /* sem UART PC não faz sentido continuar */
        return 0;
    }
    if (!device_is_ready(uart_link)) {
        pc_print("UART link não pronta!\r\n");
        return 0;
    }

    /* configurar ISRs de recepção */
    uart_irq_callback_user_data_set(uart_link, link_uart_cb, NULL);
    uart_irq_rx_enable(uart_link);

    uart_irq_callback_user_data_set(uart_pc, pc_uart_cb, NULL);
    uart_irq_rx_enable(uart_pc);

    /* configurar botão */
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

    /* loop principal (máquina de estados) */
    while (1) {

        if (current_mode == MODE_RX) {
            pc_print(">> Entrando no modo RX...\r\n");
            /* opcional: não limpar link_msgq para reter mensagens até processá-las */
            /* k_msgq_purge(&link_msgq); */

            int elapsed = 0;
            while (elapsed < RX_TIME_MS) {

                /* botão pressionado: força para o modo desejado */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    if (start_rx) {
                        pc_print("Botão pressionado -> Forçando RX (modo preferido).\r\n");
                        current_mode = MODE_RX; /* já em RX */
                    } else {
                        pc_print("Botão pressionado -> Forçando TX (modo preferido).\r\n");
                        current_mode = MODE_TX; /* pular para TX agora */
                    }
                    goto next_state;
                }

                /* Processa mensagens vindas da outra placa (link_msgq)
                 * Só imprimimos para o PC se estivermos em RX (que é o caso agora).
                 */
                while (k_msgq_get(&link_msgq, &msg, K_NO_WAIT) == 0) {
                    pc_print("[RX from other board] ");
                    pc_print(msg);
                    pc_print("\r\n");
                }

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* RX terminou normalmente -> próxima será TX */
            current_mode = MODE_TX;
        }
        else { /* MODE_TX */
            pc_print(">> Entrando no modo TX...\r\n");

            int elapsed = 0;
            while (elapsed < TX_TIME_MS) {

                /* botão pressionado: força para o modo desejado */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    if (start_rx) {
                        pc_print("Botão pressionado -> Forçando RX (modo preferido).\r\n");
                        current_mode = MODE_RX;
                    } else {
                        pc_print("Botão pressionado -> Forçando TX (modo preferido).\r\n");
                        current_mode = MODE_TX; /* já em TX */
                    }
                    goto next_state;
                }

                /* Envia TODAS as mensagens que vieram do PC durante qualquer período.
                 * Se não houver mensagem, ainda podemos enviar um keepalive opcional.
                 */
                while (k_msgq_get(&pc_msgq, &msg, K_NO_WAIT) == 0) {
                    /* encaminha para a outra placa */
                    link_send(msg);
                    link_send("\r\n");
                }

                /* opcional: keepalive ou PING caso queira sinais mesmo sem mensagens */
                /* link_send("PING\r\n"); */

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            /* TX terminou normalmente -> próxima será RX */
            current_mode = MODE_RX;
        }

    next_state:
        /* imediatamente inicia o próximo estado (não volta ao ponto anterior) */
        continue;
    }

    /* nunca chega aqui */
    return 0;
}
