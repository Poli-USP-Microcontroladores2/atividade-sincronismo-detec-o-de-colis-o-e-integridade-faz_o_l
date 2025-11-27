/* Código Integrado — Ciclo TX/RX (UART1) + Ponte de Mensagens (UART0)
 *
 * FRDM-KL25Z:
 * UART0 (Console) -> PC (Recebe mensagem do PC, Envia mensagem para o PC)
 * UART1 (Link)    -> Comunicação entre Placas (Ciclo RX/TX)
 *
 * Funcionamento:
 * 1. PC envia mensagem para Placa A (via UART0).
 * 2. Placa A armazena essa mensagem em 'pc_tx_buf'.
 * 3. Quando Placa A entra em modo TX (UART1), envia 'pc_tx_buf'.
 * 4. Placa B, em modo RX (UART1), recebe e armazena em 'link_rx_buf'.
 * 5. Quando Placa B entra em modo TX (UART1), ela envia 'link_rx_buf' para o seu PC (UART0).
 * 6. O PC B recebe a mensagem da placa A (via UART0).
 * (Comunicação bidirecional é feita de forma similar na direção oposta)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <zephyr/sys/printk.h> // Adicionado para printk no main

#define UART_PC_NODE   DT_NODELABEL(uart0)
#define UART_LINK_NODE DT_NODELABEL(uart1)

#define MSG_SIZE     32
#define CHECK_MS     100
#define RX_TIME_MS   5000
#define TX_TIME_MS   5000

/* Variáveis para comunicação entre Placas (UART1) */
K_MSGQ_DEFINE(link_msgq, MSG_SIZE, 10, 4);

/* Buffers de Mensagem */
static char link_rx_buf[MSG_SIZE]; // Armazena a mensagem recebida da outra Placa
static char pc_tx_buf[MSG_SIZE] = "Mensagem inicial Placa A"; // Armazena a mensagem a ser enviada
static int rx_pos = 0; // Posição atual no buffer de recepção (UART1)

/* Handles dos dispositivos */
static const struct device *const uart_pc   = DEVICE_DT_GET(UART_PC_NODE);
static const struct device *const uart_link = DEVICE_DT_GET(UART_LINK_NODE);

/* Botão (PTA16) - Inalterado */
const struct device *gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
#define SYNC_BUTTON_PIN 16
static struct gpio_callback button_cb_data;
K_SEM_DEFINE(sync_sem, 0, 1);

/* --- Funções de Comunicação --- */

// UART0 -> PC (usado para status e retransmissão final)
void pc_print(const char *s)
{
    while (*s) uart_poll_out(uart_pc, *s++);
}

// UART1 -> placa remota
void link_send(const char *s)
{
    while (*s) uart_poll_out(uart_link, *s++);
}

/* --- ISRs (Interrupções) --- */

// UART1 ISR (Recepção da outra Placa) - Inalterado na essência
void link_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_link)) return;
    if (!uart_irq_rx_ready(uart_link)) return;

    while (uart_fifo_read(uart_link, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (rx_pos > 0) {
                // Mensagem completa recebida da outra placa
                link_rx_buf[rx_pos] = '\0';
                k_msgq_put(&link_msgq, &link_rx_buf, K_NO_WAIT);
                rx_pos = 0;
            }
        } 
        else if (rx_pos < MSG_SIZE - 1) {
            link_rx_buf[rx_pos++] = c;
        }
    }
}

// Botão ISR - Inalterado
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
    uint8_t c;

    /* true → prefere RX (receber da outra placa) quando o botão for pressionado */
    bool start_rx = true; 
    
    /* Inicialização (Inalterado) */
    if (!device_is_ready(uart_pc))   return 0;
    if (!device_is_ready(uart_link)) return 0;

    uart_irq_callback_user_data_set(uart_link, link_uart_cb, NULL);
    uart_irq_rx_enable(uart_link);

    /* Botão (Inalterado) */
    gpio_pin_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpioa_dev, SYNC_BUTTON_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, sync_button_pressed, BIT(SYNC_BUTTON_PIN));
    gpio_add_callback(gpioa_dev, &button_cb_data);

    pc_print("Sistema iniciado.\r\n");
    pc_print(start_rx ? "Modo preferencial: RX\r\n" : "Modo preferencial: TX\r\n");

    enum { MODE_RX, MODE_TX } mode = MODE_RX;

    while (1) {
        
        /* === Ponte UART0 (PC) -> pc_tx_buf === */
        // Tenta ler do PC (UART0). Se houver dados, armazena para envio futuro.
        if (uart_poll_in(uart_pc, &c) == 0) {
            static int pc_rx_pos = 0;
            
            // Lógica para capturar a mensagem do PC até \n ou \r
            if (c == '\n' || c == '\r') {
                if (pc_rx_pos > 0) {
                    pc_tx_buf[pc_rx_pos] = '\0'; // Finaliza a string
                    pc_print("\r\n[PC] Mensagem armazenada para envio: ");
                    pc_print(pc_tx_buf);
                    pc_print("\r\n");
                    pc_rx_pos = 0; // Prepara para a próxima mensagem
                }
            } 
            else if (pc_rx_pos < MSG_SIZE - 1) {
                pc_tx_buf[pc_rx_pos++] = c;
                // Opcional: ecoar para o terminal
                uart_poll_out(uart_pc, c); 
            }
        }


        /* ======================== RX (Receber da outra Placa) ========================== */
        if (mode == MODE_RX) {

            pc_print(">> Entrando no modo RX (5s). Aguardando da Placa B...\r\n");
            k_msgq_purge(&link_msgq); // Limpa mensagens anteriores

            int elapsed = 0;
            while (elapsed < RX_TIME_MS) {

                /* Botão → força modo preferido e reinicia o ciclo (Inalterado) */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    mode = start_rx ? MODE_RX : MODE_TX;
                    pc_print("Botão pressionado → Reiniciando ciclo em ");
                    pc_print(start_rx ? "RX\r\n" : "TX\r\n");
                    break;
                }

                /* Retransmissão da Mensagem Recebida para o PC (UART0) */
                while (k_msgq_get(&link_msgq, &msg, K_NO_WAIT) == 0) {
                    // MENSAGEM RECEBIDA DA OUTRA PLACA (UART1)
                    pc_print("[LINK RX] Mensagem da outra placa: "); 
                    pc_print(msg); 
                    pc_print("\r\n");
                }

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            if (elapsed >= RX_TIME_MS)
                mode = MODE_TX;
        }

        /* ======================== TX (Enviar para a outra Placa) ========================== */
        else {

            pc_print(">> Entrando no modo TX (5s). Enviando para a Placa B...\r\n");
            int elapsed = 0;
            
            // A mensagem a ser enviada para a outra placa é o conteúdo de pc_tx_buf
            pc_print("[LINK TX] Enviando: ");
            pc_print(pc_tx_buf);
            pc_print("\r\n");

            while (elapsed < TX_TIME_MS) {

                /* Botão → força modo preferido e reinicia ciclo (Inalterado) */
                if (k_sem_take(&sync_sem, K_NO_WAIT) == 0) {
                    mode = start_rx ? MODE_RX : MODE_TX;
                    pc_print("Botão pressionado → Reiniciando ciclo em ");
                    pc_print(start_rx ? "RX\r\n" : "TX\r\n");
                    break;
                }

                // Envia a mensagem armazenada (recebida do PC)
                link_send(pc_tx_buf);
                link_send("\r\n"); // Adiciona nova linha

                k_sleep(K_MSEC(CHECK_MS));
                elapsed += CHECK_MS;
            }

            if (elapsed >= TX_TIME_MS)
                mode = MODE_RX;
        }
    }

    return 0;
}