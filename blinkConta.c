#include "pico/stdlib.h"       // Biblioteca padrão do Raspberry Pi Pico
#include "hardware/gpio.h"     // Controle de GPIOs
#include "hardware/i2c.h"      // Comunicação I2C
#include "lib/ssd1306.h"       // Biblioteca para display OLED SSD1306
#include "lib/font.h"          // Biblioteca de fontes para o display
#include "FreeRTOS.h"          // Sistema operacional FreeRTOS
#include "FreeRTOSConfig.h"    // Configurações do FreeRTOS
#include "task.h"              // Gerenciamento de tarefas FreeRTOS
#include <stdio.h>             // Funções padrão de entrada/saída
#include "hardware/pwm.h"      // Modulação por largura de pulso (PWM) para buzzer
#include "hardware/clocks.h"   // Controle de clocks do sistema
#include "ws2818b.pio.h"       // Programa PIO para controlar LEDs WS2812B
#include "hardware/pio.h"      // Interface para Programmable I/O (PIO)

// Definição dos pinos utilizados no projeto
#define LED_PIN_GREEN 11       // Pino para LED verde
#define LED_PIN_BLUE 12        // Pino para LED azul
#define LED_PIN_RED 13         // Pino para LED vermelho
#define BOTAO_A 5              // Pino para botão A
#define BOTAO_B 6              // Pino para botão B
#define BUZZER 21              // Pino para o buzzer
#define I2C_PORT i2c1          // Porta I2C utilizada
#define I2C_SDA 14             // Pino SDA para I2C
#define I2C_SCL 15             // Pino SCL para I2C
#define endereco 0x3C          // Endereço I2C do display SSD1306
#define LED_PIN 7              // Pino para matriz de LEDs WS2812B

// Variáveis globais
volatile bool modo_noturno = false; // Controla o modo noturno (true/false)
volatile int estado_semaforo = 0;   // Estado do semáforo: 0=verde, 1=amarelo, 2=vermelho
uint sm;                           // Máquina de estado do PIO para LEDs
#define LED_COUNT 25               // Número total de LEDs na matriz 5x5

// Manipulador de interrupção para botão B, usado para entrar no modo BOOTSEL
#include "pico/bootrom.h"
void gpio_irq_handler(uint gpio, uint32_t events) {
    reset_usb_boot(0, 0); // Reinicia o Pico no modo BOOTSEL para upload de firmware
}

// Inicializa os pinos GPIO usados no projeto
void init_gpios() {
    // Configura o pino do LED vermelho
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT); // Define como saída
    
    // Configura o pino do LED verde
    gpio_init(LED_PIN_GREEN);
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT); // Define como saída
    
    // Configura o pino do LED azul
    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT); // Define como saída
    
    // Configura o pino do botão A
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN); // Define como entrada
    gpio_pull_up(BOTAO_A);          // Ativa pull-up interno
    
    // Configura o pino do botão B
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN); // Define como entrada
    gpio_pull_up(BOTAO_B);          // Ativa pull-up interno
    
    // Configura o pino do buzzer
    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT); // Define como saída
}

// Estrutura para representar um pixel RGB na matriz de LEDs
struct pixel_t {
    uint8_t G, R, B;           // Componentes de cor: verde, vermelho, azul
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;       // Tipo para LEDs NeoPixel (WS2812B)

// Variáveis globais para controle da matriz de LEDs
npLED_t leds[LED_COUNT];       // Array que armazena o estado de cada LED
PIO np_pio;                    // Instância do PIO para controlar a matriz

// Matrizes que definem os padrões de exibição na matriz de LEDs (5x5 pixels)
const uint8_t digits[4][5][5][3] = {
    // Situação 1: sinal verde, seta pra cima
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 100, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 100, 0}}
    },
    // Situação 2: sinal amarelo, sinal de exclamação
    {
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Situação 3: sinal vermelho, um X
    {
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}}
    }
};

// Define as cores de um LED na matriz
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    leds[index].R = r; // Componente vermelho
    leds[index].G = g; // Componente verde
    leds[index].B = b; // Componente azul
}

// Limpa a matriz de LEDs, exibindo o padrão de dígito 4 (padrão para limpar)
void npClear() {
    npDisplayDigit(4);
}

// Inicializa a matriz de LEDs WS2812B usando o PIO
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carrega programa PIO
    np_pio = pio0; // Usa PIO0
    sm = pio_claim_unused_sm(np_pio, true); // Reserva uma máquina de estado
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f); // Inicializa PIO
    npClear(); // Limpa a matriz ao inicializar
}

// Escreve os dados dos LEDs na matriz
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G); // Envia componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].R); // Envia componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].B); // Envia componente azul
    }
    sleep_us(100); // Pequeno atraso para estabilizar a comunicação
}

// Calcula o índice de um LED na matriz com base nas coordenadas (x, y)
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linhas pares: ordem direta
    } else {
        return 24 - (y * 5 + (4 - x)); // Linhas ímpares: ordem invertida
    }
}

// Exibe um dígito ou padrão na matriz de LEDs
void npDisplayDigit(int digit) {
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna); // Calcula índice do LED
            npSetLED(posicao, digits[digit][coluna][linha][0], // Componente R
                              digits[digit][coluna][linha][1], // Componente G
                              digits[digit][coluna][linha][2]); // Componente B
        }
    }
    npWrite(); // Atualiza a matriz com os novos dados
}

// Toca um som no buzzer com frequência e duração especificadas
void play_buzzer(uint pin, uint frequency, uint duration_ms) {
    gpio_set_function(pin, GPIO_FUNC_PWM); // Configura o pino como PWM
    uint slice_num = pwm_gpio_to_slice_num(pin); // Obtém o slice PWM
    pwm_config config = pwm_get_default_config(); // Carrega configuração padrão
    // Ajusta o divisor de clock para atingir a frequência desejada
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (frequency * 4096));
    pwm_init(slice_num, &config, true); // Inicializa o PWM
    pwm_set_gpio_level(pin, 2048); // Define duty cycle (~50%)
    vTaskDelay(pdMS_TO_TICKS(duration_ms)); // Aguarda a duração do som
    pwm_set_gpio_level(pin, 0); // Desliga o buzzer
    pwm_set_enabled(slice_num, false); // Desativa o PWM
}

// Tarefa para monitorar o estado do botão A e alternar modo noturno
void vTaskChangeButtonState(void *arg) {
    bool last_state = true; // Estado inicial do botão (pull-up: alto)
    while (true) {
        bool current_state = gpio_get(BOTAO_A); // Lê estado atual do botão
        // Detecta borda de descida (botão pressionado)
        if (last_state && !current_state) {
            modo_noturno = !modo_noturno; // Alterna modo noturno
            vTaskDelay(pdMS_TO_TICKS(50)); // Debouncing de 50ms
        }
        last_state = current_state; // Atualiza estado anterior
        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeno atraso para liberar CPU
    }
}

// Tarefa para controlar o buzzer com base no modo e estado do semáforo
void vTaskBuzzer(void *arg) {
    while (true) {
        if (modo_noturno) {
            // Modo noturno: toca som longo a cada 4 segundos
            play_buzzer(BUZZER, 4000, 2000);
            vTaskDelay(pdMS_TO_TICKS(2000));
        } else {
            if (estado_semaforo == 0) {
                // Estado verde: som longo a cada 10 segundos
                play_buzzer(BUZZER, 5500, 1000);
                vTaskDelay(pdMS_TO_TICKS(5000));
            } else if (estado_semaforo == 1) {
                // Estado amarelo: som curto repetitivo
                play_buzzer(BUZZER, 5500, 300);
                vTaskDelay(pdMS_TO_TICKS(300));
            } else if (estado_semaforo == 2) {
                // Estado vermelho: som médio a cada 2 segundos
                play_buzzer(BUZZER, 5500, 500);
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
    }
}

// Tarefa para controlar o semáforo no modo diurno
void vTaskSemaforoDiurno(void *arg) {
    while (1) {
        if (!modo_noturno) {
            // Estado verde: 6 segundos
            estado_semaforo = 0;
            gpio_put(LED_PIN_GREEN, 1);
            gpio_put(LED_PIN_RED, 0);
            vTaskDelay(pdMS_TO_TICKS(6000));
            
            if (!modo_noturno) {
                // Estado amarelo: 2 segundos
                estado_semaforo = 1;
                gpio_put(LED_PIN_GREEN, 1);
                gpio_put(LED_PIN_RED, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                
                if (!modo_noturno) {
                    // Estado vermelho: 10 segundos
                    estado_semaforo = 2;
                    gpio_put(LED_PIN_GREEN, 0);
                    gpio_put(LED_PIN_RED, 1);
                    vTaskDelay(pdMS_TO_TICKS(10000));
                }
            }
        }
    }
}

// Tarefa para controlar o semáforo no modo noturno
void vTaskSemaforoNoturno() {
    while (1) {
        if (modo_noturno) {
            // Pisca LEDs verde e vermelho alternadamente a cada 2 segundos
            gpio_put(LED_PIN_GREEN, 1);
            gpio_put(LED_PIN_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            gpio_put(LED_PIN_GREEN, 0);
            gpio_put(LED_PIN_RED, 0);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

// Tarefa para controlar o display OLED SSD1306
void vTaskDisplay(void *arg) {
    // Inicializa I2C a 400 kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura pino SDA
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura pino SCL
    gpio_pull_up(I2C_SDA); // Ativa pull-up no SDA
    gpio_pull_up(I2C_SCL); // Ativa pull-up no SCL
    ssd1306_t ssd; // Estrutura para o display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa display
    ssd1306_config(&ssd); // Configura display
    ssd1306_send_data(&ssd); // Envia dados iniciais
    ssd1306_fill(&ssd, false); // Limpa o display
    ssd1306_send_data(&ssd); // Atualiza display

    char str_y[5]; // Buffer para string de tempo
    bool cor = true; // Controla cor de fundo/linha
    int ultimo_estado = -1; // Último estado do semáforo
    uint32_t tempo_inicio = 0; // Tempo de início do estado
    const uint32_t duracoes[] = {6000, 2000, 10000}; // Durações dos estados

    while (true) {
        if (modo_noturno) {
            // Modo noturno: exibe mensagem de atenção
            ssd1306_fill(&ssd, !cor); // Preenche fundo
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Desenha retângulo
            ssd1306_draw_string(&ssd, "ATENCAO", 38, 16); // Exibe "ATENCAO"
            ssd1306_line(&ssd, 3, 35, 123, 35, cor); // Desenha linha
            ssd1306_draw_string(&ssd, "NOITE", 45, 45); // Exibe "NOITE"
            ssd1306_send_data(&ssd); // Atualiza display
            vTaskDelay(pdMS_TO_TICKS(735)); // Aguarda 735ms
        } else {
            // Calcula tempo restante no estado atual
            if (estado_semaforo != ultimo_estado) {
                ultimo_estado = estado_semaforo;
                tempo_inicio = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }
            uint32_t tempo_atual = xTaskGetTickCount() * portTICK_PERIOD_MS;
            uint32_t tempo_decorrido = tempo_atual - tempo_inicio;
            uint32_t tempo_restante_ms = duracoes[estado_semaforo] > tempo_decorrido ? 
                                        duracoes[estado_semaforo] - tempo_decorrido : 0;
            int tempo_restante_seg = (tempo_restante_ms + 999) / 1000; // Arredonda para cima
            sprintf(str_y, "%d", tempo_restante_seg); // Converte tempo para string
            
            if (estado_semaforo == 0) {
                // Estado verde: exibe instruções para prosseguir
                ssd1306_fill(&ssd, !cor);
                ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
                ssd1306_line(&ssd, 3, 25, 123, 25, cor);
                ssd1306_line(&ssd, 3, 37, 123, 37, cor);
                ssd1306_draw_string(&ssd, "SINAL VERDE", 20, 10);
                ssd1306_draw_string(&ssd, "SIGA", 50, 28);
                ssd1306_draw_string(&ssd, "TEMPO:", 20, 45);
                ssd1306_draw_string(&ssd, str_y, 80, 45);
                ssd1306_send_data(&ssd);
            } else if (estado_semaforo == 1) {
                // Estado amarelo: exibe alerta
                ssd1306_fill(&ssd, !cor);
                ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
                ssd1306_line(&ssd, 3, 25, 123, 25, cor);
                ssd1306_line(&ssd, 3, 37, 123, 37, cor);
                ssd1306_draw_string(&ssd, "SINAL AMARELO", 12, 10);
                ssd1306_draw_string(&ssd, "ATENCAO", 40, 28);
                ssd1306_draw_string(&ssd, "TEMPO:", 20, 45);
                ssd1306_draw_string(&ssd, str_y, 80, 45);
                ssd1306_send_data(&ssd);
            } else if (estado_semaforo == 2) {
                // Estado vermelho: exibe instrução para parar
                ssd1306_fill(&ssd, !cor);
                ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
                ssd1306_line(&ssd, 3, 25, 123, 25, cor);
                ssd1306_line(&ssd, 3, 37, 123, 37, cor);
                ssd1306_draw_string(&ssd, "SINAL VERMELHO", 8, 10);
                ssd1306_draw_string(&ssd, "PARE", 50, 28);
                ssd1306_draw_string(&ssd, "TEMPO:", 20, 45);
                ssd1306_draw_string(&ssd, str_y, 80, 45);
                ssd1306_send_data(&ssd);
            }
            vTaskDelay(pdMS_TO_TICKS(735)); // Aguarda 735ms
        }
    }
}

// Tarefa para atualizar a matriz de LEDs
void vTaskMatrizDeLeds(void *pvParameters) {
    while (1) {
        int local_estado = estado_semaforo; // Cópia local do estado
        bool local_modo_noturno = modo_noturno; // Cópia local do modo
        if (!local_modo_noturno) {
            npDisplayDigit(local_estado); // Exibe estado do semáforo
        } else {
            npDisplayDigit(1); // Exibe padrão noturno
        }
        vTaskDelay(pdMS_TO_TICKS(150)); // Aguarda 150ms
    }
}

// Função principal do programa
int main() {
    stdio_init_all(); // Inicializa comunicação serial
    init_gpios(); // Configura os GPIOs
    
    // Configura interrupção para botão B (modo BOOTSEL)
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    npInit(LED_PIN); // Inicializa a matriz de LEDs WS2812B
    
    // Cria as tarefas do FreeRTOS
    xTaskCreate(vTaskChangeButtonState, "Button", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskSemaforoDiurno, "Diurno", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskSemaforoNoturno, "Noturno", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskBuzzer, "Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskDisplay, "Display", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskMatrizDeLeds, "Matriz", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    
    vTaskStartScheduler(); // Inicia o escalonador FreeRTOS
    panic_unsupported(); // Em caso de falha no escalonador
}