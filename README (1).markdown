# Semáforo Inteligente 🚦

## Descrição 📝
Projeto de um semáforo inteligente implementado no Raspberry Pi Pico, utilizando FreeRTOS, display OLED SSD1306, matriz de LEDs WS2812B e buzzer. O sistema alterna entre modos diurno ☀️ e noturno 🌙, controlado por um botão, exibindo estados do semáforo (verde, amarelo, vermelho) e mensagens no display.

## Funcionalidades ✨
- **Modo diurno** ☀️: Ciclo de semáforo com LEDs, buzzer e contagem regressiva no display.
- **Modo noturno** 🌙: LEDs piscantes e mensagem de atenção no display.
- **Botão A** 🟢: Alterna entre modos diurno e noturno.
- **Botão B** 🔵: Entra no modo BOOTSEL para upload de firmware.
- **Matriz de LEDs** 💡: Exibe padrões visuais para cada estado.

## Requisitos 🛠️
- Raspberry Pi Pico 🖥️
- Display OLED SSD1306 (I2C) 📺
- Matriz de LEDs WS2812B (5x5) 🌈
- Buzzer 🔊
- 2 botões 🎮
- SDK do Pico e FreeRTOS ⚙️

## Instruções 🚀
1. Conecte os componentes conforme os pinos definidos em `main.c` 🔌.
2. Compile e carregue o código no Raspberry Pi Pico 💾.
3. Pressione o botão A para alternar modos; botão B para modo BOOTSEL 🕹️.

## Arquivos 📂
- `main.c`: Código principal com lógica do semáforo e tarefas FreeRTOS 🧠.
- `ws2818b.pio`: Programa PIO para controle da matriz de LEDs 🌟.
- `lib/ssd1306.h` e `lib/font.h`: Bibliotecas para o display OLED 🖼️.

## Autor 👨‍💻
[Seu Nome]