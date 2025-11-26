/*
 * FRDM-KL25Z — Dual-UART, comportamento solicitado
 *
 * UART0 (PC <-> placa):  PTA1 RX, PTA2 TX
 * UART1 (Placa <-> placa): PTE1 RX, PTE0 TX
 *
 * PLATE_A = 1  -> Placa A: PC -> (store in RX) -> TX -> envia via UART1
 * PLATE_A = 0  -> Placa B: UART1 -> (store in RX) -> TX -> envia ao PC
 *
 * Ciclo contínuo: RX_TIME_MS (5s) -> TX_TIME_MS (5s) -> repete
 * Botão (PTA16) força a placa a entrar no modo determinado por start_rx.
 * Após forçar, a placa continua o ciclo a partir do novo modo (reset do tempo).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#define PLATE_A 1           /* Defina 1 para PLACA A, 0 para PLACA B */
#define UART_PC_NODE   DT_NODELABEL(uart0)
#define UART_LINK_NODE DT_NODELABEL(uart1)

#define MSG_SIZE    64
#define CHECK_MS    100
#define RX_TIME_MS  5000
#define TX_TIME_MS  5000

/* Filas */
K_MSGQ_DEFINE(pc_to_link_q, MSG_SIZE, 10, 4);   /* Mensagens vindas do PC (para enviar no TX - útil para PLATE_A) */
K_MSGQ_DEFINE(link_to_pc_q, MSG_SIZE, 10, 4);   /* Mensagens vindas da outra placa (para enviar ao PC no TX - útil para PLATE_B) */

/* UART devices */
static const struct device *const uart_pc   = DEVICE_DT_GET(UART_PC_NODE);   /* UART0 */
static const struct device *const uart_link = DEVICE_DT_GET(UART_LINK_NODE); /* UART1 */

/* buffers de ISR */
static char pc_rx_buf[MSG_SIZE];
static int  pc_rx_pos = 0;

static char link_rx_buf[MSG_SIZE];
static int  link_rx_pos = 0;

/* Botão */
const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define SYNC_BUTTON_PIN 16
static struct gpio_callback button_cb_data;
K_SEM_DEFINE(sync_sem, 0, 1);

/* Estado atual do ciclo */
static bool current_is_rx = true;   /* começa em RX por padrão */
static bool start_rx = true;        /* preferencia: se true => botão força RX; se false => força TX */

/* ------------------------------------------------------------------ */
/* UART0 ISR: recebe do PC — aceita SOMENTE se estamos em RX (coleta)   */
/* ------------------------------------------------------------------ */
void uart_pc_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_pc)) return;
    if (!uart_irq_rx_ready(uart_pc)) return;

    while (uart_fifo_read(uart_pc, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (pc_rx_pos > 0) {
                pc_rx_buf[pc_rx_pos] = '\0';
                /* Só guarda a mensagem se estamos em RX */
                if (current_is_rx) {
                    /* Enfileira para ser enviada pela UART_LINK durante o próximo TX (essencialmente Plate A behavior) */
                    k_msgq_put(&pc_to_link_q, &pc_rx_buf, K_NO_WAIT);
                }
                pc_rx_pos = 0;
            }
        } else if (pc_rx_pos < (MSG_SIZE - 1)) {
            pc_rx_buf[pc_rx_pos++] = c;
        }
    }
}

/* ------------------------------------------------------------------ */
/* UART1 ISR: recebe da outra placa — aceita SOMENTE se estamos em RX  */
/* ------------------------------------------------------------------ */
void uart_link_isr(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_link)) return;
    if (!uart_irq_rx_ready(uart_link)) return;

    while (uart_fifo_read(uart_link, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (link_rx_pos > 0) {
                link_rx_buf[link_rx_pos] = '\0';
                /* Só aceita mensagens do link quando estiver em RX */
                if (current_is_rx) {
                    k_msgq_put(&link_to_pc_q, &link_rx_buf, K_NO_WAIT);
                }
                link_rx_pos = 0;
            }
        } else if (link_rx_pos < (MSG_SIZE - 1)) {
            link_rx_pos++;
            link_rx_buf[link_rx_pos - 1] = c;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Funções de envio (bloqueantes simples)                             */
/* ------------------------------------------------------------------ */
void pc_print(const char *s)
{
    while (*s) uart_poll_out(uart_pc, *s++);
}

void link_send(const char *s)
{
    while (*s) uart_poll_out(uart_link, *s++);
}

/* ------------------------------------------------------------------ */
/* Botão callback                                                     */
/* ------------------------------------------------------------------ */
void sync_button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /* sinaliza que o botão foi pressionado */
    k_sem_give(&sync_sem);
}

/* ------------------------------------------------------------------ */
/* MAIN                                                                 */
/* ------------------------------------------------------------------ */
int main(void)
{
    char tmp[MSG_SIZE];

    /* Ajuste manual: qual modo o botão deve forçar? */
#if PLATE_A
    /* Para PLACA A: o fluxo típico é PC -> (armazenar em RX) -> TX -> enviar via link */
    start_rx = true;   /* Exemplo: botão força RX (ajuste se quiser) */
#else
    /* Para PLACA B: o fluxo típico é receber pelo link em RX e depois enviar ao PC em TX */
    start_rx = true;   /* ajuste conforme preferir */
#endif

    /* Verifica UARTs */
    if (!device_is_ready(uart_pc)) {
        /* Se não estiver pronto, não prosseguir */
        return 0;
    }
    if (!device_is_ready(uart_link)) {
        return 0;
    }

    /* Configura ISRs */
    uart_irq_callback_user_data_set(uart_pc, uart_pc_isr, NULL);
    uart_irq_rx_enable(uart_pc);

    uart_irq_callback_user_data_set(uart_link, uart_link_isr, NULL);
    uart_irq_rx_enable(uart_link);

    /* Configura botão */
    if (!device_is_ready(gpioa_dev)) {
        return 0;
    }

    gpio_pin_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, sync_button_pressed, BIT(SYNC_BUTTON_PIN));
    gpio_add_callback(gpioa_dev, &button_cb_data);

    pc_print("Sistema iniciado.\r\n");
#if PLATE_A
    pc_print("PLATE_A: comportamento PC -> link\n");
#else
    pc_print("PLATE_B: comportamento link -> PC\n");
#endif

    /* Inicia ciclo em RX */
    current_is_rx = true;
    int elapsed = 0;

    while (1) {
        /* Se o botão for pressionado: força o modo definido por start_rx e reinicia o contador */
        if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
            current_is_rx = start_rx;
            elapsed = 0;
            if (current_is_rx) pc_print("[BOTAO] Forcado: RX\n");
            else                pc_print("[BOTAO] Forcado: TX\n");
            /* continua no novo modo imediatamente */
        }

        if (current_is_rx) {
            /* MODO RX: coleta mensagens (ISRs já enfileiram quando em RX) */
            pc_print(">> MODO RX ativo\n");

            /* Durante RX, imprime no PC o que foi recebido do link (apenas se houver) */
            int loop_ms = 0;
            while (loop_ms < RX_TIME_MS) {
                /* se botão pressionado, forçar novo modo e reiniciar */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    current_is_rx = start_rx;
                    elapsed = 0;
                    if (current_is_rx) pc_print("[BOTAO] Forcado: RX\n");
                    else                pc_print("[BOTAO] Forcado: TX\n");
                    break; /* sai RX e continua no novo modo */
                }

                /* Para PLATE_B: mensagens vindas do link são enfileiradas em link_to_pc_q dentro do ISR
                 * Exibimos (ou apenas confirmamos) para o PC local */
                while (k_msgq_get(&link_to_pc_q, &tmp, K_NO_WAIT) == 0) {
                    /* PARA PLATE_B: recebemos dado da outra placa enquanto estamos em RX */
                    pc_print("[RX->PC_QUEUE] "); pc_print(tmp); pc_print("\r\n");
                }

                /* Para PLATE_A: mensagens vindas do PC são enfileiradas em pc_to_link_q dentro do ISR
                 * Aqui poderíamos confirmar recebimento ao PC, se desejado. */
                while (k_msgq_get(&pc_to_link_q, &tmp, K_NO_WAIT) == 0) {
                    /* PLATE_A recebeu algo do PC enquanto em RX; deixamos a fila para enviar quando entrarmos em TX */
                    pc_print("[PC->STORE] "); pc_print(tmp); pc_print("\r\n");
                    /* re-enfileira: como pegamos da fila para mostrar, colocamos de volta */
                    k_msgq_put(&pc_to_link_q, &tmp, K_NO_WAIT);
                    break; /* mostramos apenas o item do momento */
                }

                k_sleep(K_MSEC(CHECK_MS));
                loop_ms += CHECK_MS;
            }

            /* terminou periodo RX sem interrupção: passa a TX */
            if (loop_ms >= RX_TIME_MS) {
                current_is_rx = false;
                elapsed = 0;
            }
        } else {
            /* MODO TX: envia as mensagens armazenadas de acordo com o papel da placa */
            pc_print(">> MODO TX ativo\n");

            int loop_ms = 0;
            while (loop_ms < TX_TIME_MS) {
                /* botão -> força */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    current_is_rx = start_rx;
                    elapsed = 0;
                    if (current_is_rx) pc_print("[BOTAO] Forcado: RX\n");
                    else                pc_print("[BOTAO] Forcado: TX\n");
                    break; /* sai TX e vai ao modo forçado */
                }

#if PLATE_A
                /* PLACA A: enviar para a outra placa tudo que foi recebido do PC durante RX */
                while (k_msgq_get(&pc_to_link_q, &tmp, K_NO_WAIT) == 0) {
                    /* envia via UART1 */
                    link_send(tmp);
                    link_send("\r\n");
                    pc_print("[TX->LINK] "); pc_print(tmp); pc_print("\r\n");
                }
#else
                /* PLACA B: enviar para o PC tudo que foi recebido da outra placa durante RX */
                while (k_msgq_get(&link_to_pc_q, &tmp, K_NO_WAIT) == 0) {
                    /* envia via UART0 (PC) */
                    pc_print(tmp);
                    pc_print("\r\n");
                    pc_print("[TX->PC] "); pc_print(tmp); pc_print("\r\n");
                }
#endif

                /* Se não houve mensagens, ainda dormimos pequenos intervalos para checar botão */
                k_sleep(K_MSEC(CHECK_MS));
                loop_ms += CHECK_MS;
            }

            /* terminou TX sem interrupção: passa a RX */
            if (loop_ms >= TX_TIME_MS) {
                current_is_rx = true;
                elapsed = 0;
            }
        }
    }

    return 0;
}
