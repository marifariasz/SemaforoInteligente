#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include "hardware/pwm.h"      // Modulação por largura de pulso
#include "hardware/clocks.h"   // Controle de clocks
#include "ws2818b.pio.h"       // Programa PIO para LEDs WS2812B
#include "hardware/pio.h"

// GPIOs usados
#define LED_PIN_GREEN 11
#define LED_PIN_BLUE 12
#define LED_PIN_RED 13
#define BOTAO_A 5
#define BOTAO_B 6
#define BUZZER 21
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

#define LED_PIN 7              // Pino para a matriz de LEDs WS2812B

// Variáveis globais
volatile bool modo_noturno = false;
volatile int estado_semaforo = 0; // 0 = verde, 1 = amarelo, 2 = vermelho
uint sm;                       // Máquina de estado do PIO
#define LED_COUNT 25           // Número de LEDs na matriz


// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

// Inicializa GPIOs
void init_gpios() {

    // INICIALIZANDO O PINO DO LED VERMELHO
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);

    // INICIALIZANDO O PINO DO LED VERDE
    gpio_init(LED_PIN_GREEN);
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);

    // INICIALIZANDO O PINO DO LED AZUL
    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);

    // INICIALIZANDO O PINO DO BOTAO A
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);

    // INICIALIZANDO O PINO DO BOTAO B
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);

    // INICIALIZANDO O PINO DO BUZZER
    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT);
}


void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b);
void npClear();
void npInit(uint pin);
void npWrite();
void npDisplayDigit(int digit);
// Estrutura para representar um pixel RGB na matriz de LEDs
struct pixel_t {
    uint8_t G, R, B;           // Componentes verde, vermelho e azul
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;       // Tipo para LEDs NeoPixel

// Variáveis globais
npLED_t leds[LED_COUNT];       // Array para armazenar estado dos LEDs
PIO np_pio;                    // Instância do PIO para controle da matriz de LEDs

// Matrizes para exibição de dígitos na matriz de LEDs (5x5 pixels, RGB)
// Cada dígito/situação é representado por uma matriz de cores
const uint8_t digits[4][5][5][3] = {
    // Situação 1 (Rodoviária): Linha superior roxa, um pixel azul na última linha
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 100, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 100, 0}}
    },
    // Dígito 1: Linha superior roxa, pixel azul na penúltima linha
    {
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 100, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Dígito 2: Linha superior roxa, pixel azul na terceira linha
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
    leds[index].R = r; // Define componente vermelho
    leds[index].G = g; // Define componente verde
    leds[index].B = b; // Define componente azul
}

// Limpa a matriz de LEDs, exibindo o dígito 10 (padrão para limpar)
void npClear() {
    npDisplayDigit(4);
}

// Inicializa a matriz de LEDs WS2812B usando PIO
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carrega programa PIO
    np_pio = pio0; // Usa PIO0
    sm = pio_claim_unused_sm(np_pio, true); // Reserva máquina de estado
    // Inicializa programa PIO com pino e frequência
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
    npClear(); // Limpa a matriz
}

// Escreve os dados dos LEDs na matriz
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        // Envia componentes G, R, B para o PIO em sequência
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100); // Pequeno atraso para estabilizar
}

// Calcula o índice de um LED na matriz com base em coordenadas (x, y)
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linhas pares: ordem direta
    } else {
        return 24 - (y * 5 + (4 - x)); // Linhas ímpares: ordem invertida
    }
}

// Exibe um dígito na matriz de LEDs
void npDisplayDigit(int digit) {
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna); // Calcula índice do LED
            // Define cores do LED com base na matriz de dígitos
            npSetLED(
                posicao,
                digits[digit][coluna][linha][0], // R
                digits[digit][coluna][linha][1], // G
                digits[digit][coluna][linha][2]  // B
            );
        }
    }
    npWrite(); // Escreve na matriz
}

// Toca um som no buzzer com frequência e duração especificadas
void play_buzzer(uint pin, uint frequency, uint duration_ms) {
    gpio_set_function(pin, GPIO_FUNC_PWM); // Configura pino como PWM
    uint slice_num = pwm_gpio_to_slice_num(pin); // Obtém slice PWM
    pwm_config config = pwm_get_default_config(); // Configuração padrão
    // Ajusta divisor de clock para atingir a frequência desejada
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (frequency * 4096));
    pwm_init(slice_num, &config, true); // Inicializa PWM
    pwm_set_gpio_level(pin, 2048); // Define duty cycle de ~50%
    vTaskDelay(pdMS_TO_TICKS(duration_ms)); // Aguarda a duração especificada
    pwm_set_gpio_level(pin, 0); // Desliga o som
    pwm_set_enabled(slice_num, false); // Desativa PWM
}

void vTaskChangeButtonState(void *arg){
    bool last_state = true; // Botão com pull-up (alto quando não pressionado)

    while (true) {
        bool current_state = gpio_get(BOTAO_A);
        // Verifica se o estado anterior era 1 e se o estado atual é 0, que caracteriza uma Borda de Descida (Botão pressionado)
        if (last_state && !current_state) { 
            modo_noturno = !modo_noturno; // Alterna modo
            vTaskDelay(pdMS_TO_TICKS(50)); // Debouncing de 50ms
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10)); // Atraso para liberar CPU
    }
}

void vTaskBuzzer(void *arg){
    while(true){
        if(modo_noturno){
            play_buzzer(BUZZER, 4000, 2000);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }else{
            if(estado_semaforo == 0){
               play_buzzer(BUZZER, 5500, 1000);
               vTaskDelay(pdMS_TO_TICKS(5000));
            }else if(estado_semaforo == 1){
                play_buzzer(BUZZER, 5500, 300);
                vTaskDelay(pdMS_TO_TICKS(300));
            }else if(estado_semaforo == 2){
                play_buzzer(BUZZER, 5500, 500);
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
    }
}

void vTaskSemaforoDiurno(void *arg){
    while(1){
        if(modo_noturno == 0){
            // SINAL VERDE
            estado_semaforo = 0;
            gpio_put(LED_PIN_GREEN, 1);
            gpio_put(LED_PIN_RED, 0);
            vTaskDelay(pdMS_TO_TICKS(6000));

            if(modo_noturno == 0){
                // SINAL AMARELO
                estado_semaforo = 1;
                gpio_put(LED_PIN_GREEN, 1);
                gpio_put(LED_PIN_RED, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));

                if(modo_noturno == 0){
                    // SINAL VERMELHO
                    estado_semaforo = 2;
                    gpio_put(LED_PIN_GREEN, 0);
                    gpio_put(LED_PIN_RED, 1);
                    vTaskDelay(pdMS_TO_TICKS(10000));
                }
            }    
        }
    }
}

void vTaskSemaforoNoturno(){
    while(1){
        if(modo_noturno == true){
            gpio_put(LED_PIN_GREEN, 1);
            gpio_put(LED_PIN_RED, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            gpio_put(LED_PIN_GREEN, 0);
            gpio_put(LED_PIN_RED, 0);
            vTaskDelay(pdMS_TO_TICKS(2000));
        } 
    }
}
void vTaskDisplay(void *arg)
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_t ssd;                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    char str_y[5]; // Buffer para armazenar a string
    bool cor = true;
    int ultimo_estado = -1; // Último estado do semáforo
    uint32_t tempo_inicio = 0; // Tempo de início do estado (em ms)
    const uint32_t duracoes[] = {6000, 2000, 10000}; // Durações dos estados (verde, amarelo, vermelho)

    while (true)
    {
        if(modo_noturno){

            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
            ssd1306_draw_string(&ssd, "ATENCAO", 38, 16);      // Desenha uma string
            ssd1306_line(&ssd, 3, 35, 123, 35, cor);           // Desenha uma linha
            ssd1306_draw_string(&ssd, "NOITE", 45, 45);        // Desenha uma string
            ssd1306_send_data(&ssd);                           // Atualiza o display
            vTaskDelay(pdMS_TO_TICKS(735));
        } else{


            // Verifica se o estado mudou
                        if (estado_semaforo != ultimo_estado) {
                            ultimo_estado = estado_semaforo;
                            tempo_inicio = xTaskGetTickCount() * portTICK_PERIOD_MS; // Marca o início do estado
                        }
            
                        // Calcula o tempo restante
                        uint32_t tempo_atual = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        uint32_t tempo_decorrido = tempo_atual - tempo_inicio;
                        uint32_t tempo_restante_ms = duracoes[estado_semaforo] > tempo_decorrido ? 
                                                    duracoes[estado_semaforo] - tempo_decorrido : 0;
                        int tempo_restante_seg = (tempo_restante_ms + 999) / 1000; // Arredonda pra cima
            if(estado_semaforo == 0 && modo_noturno == false){
                sprintf(str_y, "%d", tempo_restante_seg); // Converte em string
                ssd1306_fill(&ssd, !cor);                          // Limpa o display
                ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
                ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
                ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
                ssd1306_draw_string(&ssd, "SINAL VERDE", 20, 10);  // Desenha uma string
                ssd1306_draw_string(&ssd, "SIGA", 50, 28); // Desenha uma string
                ssd1306_draw_string(&ssd, "TEMPO:", 20, 45);    // Desenha uma string
                ssd1306_draw_string(&ssd, str_y, 80, 45);          // Desenha uma string
                ssd1306_send_data(&ssd);                           // Atualiza o display
                vTaskDelay(pdMS_TO_TICKS(735));
            }else if(estado_semaforo == 1 && modo_noturno == false){
                sprintf(str_y, "%d", tempo_restante_seg); // Converte em string
                ssd1306_fill(&ssd, !cor);                          // Limpa o display
                ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
                ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
                ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
                ssd1306_draw_string(&ssd, "SINAL AMARELO", 12, 10);  // Desenha uma string
                ssd1306_draw_string(&ssd, "ATENCAO", 40, 28); // Desenha uma string
                ssd1306_draw_string(&ssd, "TEMPO:", 20, 45);    // Desenha uma string
                ssd1306_draw_string(&ssd, str_y, 80, 45);          // Desenha uma string
                ssd1306_send_data(&ssd);                           // Atualiza o display
                vTaskDelay(pdMS_TO_TICKS(735));
            }else if(estado_semaforo == 2 && modo_noturno == false){
                sprintf(str_y, "%d", tempo_restante_seg); // Converte em string
                ssd1306_fill(&ssd, !cor);                          // Limpa o display
                ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
                ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
                ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
                ssd1306_draw_string(&ssd, "SINAL VERMELHO", 8, 10);  // Desenha uma string
                ssd1306_draw_string(&ssd, "PARE", 50, 28); // Desenha uma string
                ssd1306_draw_string(&ssd, "TEMPO:", 20, 45);    // Desenha uma string
                ssd1306_draw_string(&ssd, str_y, 80, 45);          // Desenha uma string
                ssd1306_send_data(&ssd);                           // Atualiza o display
                vTaskDelay(pdMS_TO_TICKS(735));
            }
           // contador = 21;
        }
    }
}


void vTaskMatrizDeLeds(void *pvParameters) {
    while (1) {
        int local_estado = estado_semaforo; // Cópia local
        bool local_modo_noturno = modo_noturno; // Cópia local
        if (!local_modo_noturno) {
            npDisplayDigit(local_estado);
        } else {
            npDisplayDigit(1);
        }
        vTaskDelay(pdMS_TO_TICKS(150)); // Atraso para liberar CPU
    }
}

int main() {

    stdio_init_all();

    init_gpios();
    // Para ser utilizado o modo BOOTSEL com botão B

    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL,
                                         true, &gpio_irq_handler);
    // Fim do trecho para modo BOOTSEL com botão B
    // Inicializa matriz de LEDs
    npInit(LED_PIN);

    xTaskCreate(vTaskChangeButtonState, "Button", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTaskSemaforoDiurno, "Diurno", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskSemaforoNoturno, "Noturno", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskBuzzer, "Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskDisplay, "Display", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskMatrizDeLeds, "Matriz", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    
    // Inicia o escalonador FreeRTOS
    vTaskStartScheduler();
    panic_unsupported(); // Em caso de erro 
}