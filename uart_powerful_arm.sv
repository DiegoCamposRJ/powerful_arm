// uart_powerful_arm.sv
// V3: Adicionado botão (T3) para enviar dados do Endereço 0
//
// =============================================================================
// MÓDULO PRINCIPAL: uart_powerful_arm
//
// DESCRIÇÃO:
// Este módulo implementa uma memória de posições de servo (braço robótico)
// controlada via UART. Ele pode:
// 1. SALVAR (Escrever) 4 valores de servo (Base, Altura, Angulo, Garra) em
//    um endereço de memória (0-255), usando o comando 0xA0.
// 2. LER (Reproduzir) os 4 valores de um endereço de memória, usando o
//    comando 0xB0, e transmiti-los de volta pela UART.
// 3. REPRODUZIR (via Botão) os dados do Endereço 0 ao pressionar o botão
//    físico btn_send_n (T3).
//
// LEDs de STATUS:
// - led_grava_n (D2): Pisca (fica 0) quando uma gravação é concluída.
// - led_envio_n (C1): Acende (fica 0) enquanto a UART está transmitindo.
// - led_reproducao_n (C2): Acende (fica 0) durante tod/o processo de leitura.
// =============================================================================

module uart_powerful_arm #(
    parameter clk_freq = 25_000_000, // Frequência do clock de entrada (25 MHz)
    parameter baud_rate = 115200     // Taxa de comunicação UART
)(
    // --- Portas do Sistema ---
    input  logic       clk,        // Clock principal (P3)
    input  logic       reset_n,    // Reset ativo-baixo (D1)
    
    // --- Portas de Entrada do Usuário ---
    input  logic       btn_send_n, // Botão de "Enviar Posição 0" (E3)
    input  logic       uart_rx,    // Pino de recebimento UART (A3)
    
    // --- Portas de Saída ---
    output logic       uart_tx,        // Pino de transmissão UART (E2)
    output logic       led_grava_n,    // LED de status "Gravação" (D2)
    output logic       led_envio_n,    // LED de status "Envio" (C1)
    output logic       led_reproducao_n // LED de status "Reprodução" (C2)
);

    // --- Sinais de Interface UART ---
    // Sinais internos que conectam esta FSM ao módulo uart_top
    logic       rx_dv;       // '1' por um ciclo quando um byte é recebido (Data Valid)
    logic [7:0] rx_byte;     // O byte que foi recebido
    logic       tx_dv;       // '1' por um ciclo para dizer ao uart_top para enviar um byte
    logic [7:0] tx_byte;     // O byte que queremos enviar
    logic       tx_active;   // '1' enquanto o uart_top está ocupado transmitindo
    
    // --- Comandos do Protocolo ---
    // Define os bytes mágicos para os comandos
    localparam CMD_SAVE_POS = 8'hA0; // Comando para Salvar Posição
    localparam CMD_READ_POS = 8'hB0; // Comando para Ler Posição

    // --- Memória (BRAM) ---
    // Define a memória principal.
    // É um array de 256 posições (endereços 0 a 255).
    // Cada posição contém um array de 4 bytes (para Base, Altura, Angulo, Garra).
    logic [7:0] servo_memory [255:0] [3:0];
    
    // --- Lógica do Botão (Debouncer e Edge Detect) ---
    // Filtra o ruído mecânico do botão físico
    localparam DEBOUNCE_CYCLES = clk_freq / 100; // Define o tempo de debounce (10ms)
    logic [$clog2(DEBOUNCE_CYCLES):0] debounce_counter; // Contador para o debounce
    logic btn_send_stable;  // Estado estável (filtrado) do botão
    logic btn_send_n_sync;  // Botão sincronizado com o clock
    logic btn_send_prev;    // Estado anterior do botão (para detectar borda)
    logic btn_pressed_pulse; // Pulso de 1 ciclo quando botão é pressionado (1->0)

    // Sincronizador de 2 flops para entrada assíncrona do botão
    // (Previne metaestabilidade)
    always_ff @(posedge clk) begin
         btn_send_n_sync <= btn_send_n;
    end

    // Lógica do Debouncer
    // Só atualiza o estado estável (btn_send_stable) se o sinal
    // de entrada (btn_send_n_sync) permanecer diferente por DEBOUNCE_CYCLES.
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            btn_send_stable <= 1'b1; // Botão solto (Pull-up)
            debounce_counter <= '0;
        end else if (btn_send_stable != btn_send_n_sync) begin
            // O sinal mudou, reinicia o contador
            debounce_counter <= DEBOUNCE_CYCLES;
        end else if (debounce_counter > 0) begin
            // O sinal está estável, mas o tempo ainda não acabou
            debounce_counter <= debounce_counter - 1;
        end else begin
            // Tempo de debounce acabou, atualiza o estado
            btn_send_stable <= btn_send_n_sync;
        end
    end

    // Edge Detector (detecta borda de descida: 1 -> 0)
    // Compara o estado estável atual com o estado anterior.
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) btn_send_prev <= 1'b1;
        else btn_send_prev <= btn_send_stable;
    end
    // Gera um pulso de 1 ciclo se o estado anterior era '1' (solto)
    // e o estado atual é '0' (pressionado).
    assign btn_pressed_pulse = btn_send_prev & !btn_send_stable;
    
    // --- Registradores da FSM ---
    // Define os estados da Máquina de Estados Finitos (FSM)
    typedef enum logic [3:0] {
        IDLE,           // 0: Estado de espera
        // Estados de Salvar (Write)
        S_GET_ADDR,     // 1: Espera byte de Endereço
        S_GET_BASE,     // 2: Espera byte da Base
        S_GET_ALTURA,   // 3: Espera byte da Altura
        S_GET_ANGULO,   // 4: Espera byte do Ângulo
        S_GET_GARRA,    // 5: Espera byte da Garra
        // Estados de Ler (Read/Reprodução)
        R_GET_ADDR,     // 6: Espera byte de Endereço (para ler)
        R_PREP_DATA,    // 7: Prepara os dados (lê da BRAM para registradores)
        R_SEND_BASE,    // 8: Envia byte da Base
        R_SEND_ALTURA,  // 9: Envia byte da Altura
        R_SEND_ANGULO,  // 10: Envia byte do Ângulo
        R_SEND_GARRA,   // 11: Envia byte da Garra
        R_WAIT_TX_BUSY  // 12: Espera o módulo UART terminar de enviar o byte
    } state_t;
    
    state_t state, next_state_on_tx_done; // Estado atual e próximo estado (usado pelo TX)
    
    // Registradores temporários para os dados
    logic [7:0] reg_addr, reg_base, reg_altura, reg_angulo, reg_garra;
    logic       save_trigger; // Pulso de 1 ciclo para dizer à BRAM para salvar
    
    // Instancia o módulo UART (o "motor" da comunicação)
    uart_top #(
        .CLK_FREQ_HZ(clk_freq),
        .BAUD_RATE(baud_rate)
    ) uart_inst (
        .i_clk(clk), .i_rst_n(reset_n),
        .i_uart_rx(uart_rx), .o_uart_tx(uart_tx),
        .i_tx_dv(tx_dv), .i_tx_byte(tx_byte),
        .o_tx_active(tx_active), .o_tx_done(),
        .o_rx_dv(rx_dv), .o_rx_byte(rx_byte)
    );
    
    // --- Lógica de Controle (FSM) ---
    // Define os valores padrão para as saídas
    assign tx_dv = 1'b0;   // Não envia nada, a menos que um estado R_SEND_... o ative
    assign tx_byte = 8'h00; // Valor padrão
    
    // Bloco principal da FSM
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin // Lógica de Reset
            state <= IDLE;
            save_trigger <= 1'b0;
            next_state_on_tx_done <= IDLE;
        end else begin
            // Reseta o pulso de 'save_trigger' a cada ciclo
            save_trigger <= 1'b0;
            
            case (state)
                IDLE: begin
                    // Prioridade 1: Verifica se há um comando chegando pela UART
                    if (rx_dv) begin
                        if (rx_byte == CMD_SAVE_POS) state <= S_GET_ADDR;
                        if (rx_byte == CMD_READ_POS) state <= R_GET_ADDR;
                    // Prioridade 2: Se a UART estiver ociosa, verifica o botão
                    end else if (btn_pressed_pulse) begin
                        // Botão pressionado!
                        // Força o endereço de leitura para 0
                        reg_addr <= 8'h00; 
                        // Pula direto para a preparação de envio
                        state <= R_PREP_DATA; 
                    end
                end
                
                // --- Sequência de SALVAR (Recebimento de 5 bytes) ---
                S_GET_ADDR:   if (rx_dv) begin reg_addr <= rx_byte;   state <= S_GET_BASE;   end
                S_GET_BASE:   if (rx_dv) begin reg_base <= rx_byte;   state <= S_GET_ALTURA; end
                S_GET_ALTURA: if (rx_dv) begin reg_altura <= rx_byte; state <= S_GET_ANGULO; end
                S_GET_ANGULO: if (rx_dv) begin reg_angulo <= rx_byte; state <= S_GET_GARRA;  end
                S_GET_GARRA:  if (rx_dv) begin // Último byte da sequência
                    reg_garra <= rx_byte;
                    save_trigger <= 1'b1; // Aciona o salvamento na BRAM e o LED
                    state <= IDLE;        // Retorna ao estado de espera
                end
                
                // --- Sequência de LER (Iniciada por UART) ---
                R_GET_ADDR: begin
                    if (rx_dv) begin
                        reg_addr <= rx_byte;  // Armazena o endereço a ser lido
                        state <= R_PREP_DATA; // Vai preparar os dados
                    end
                end
                
                // --- Sequência de REPRODUÇÃO (Iniciada por UART ou Botão) ---
                R_PREP_DATA: begin
                    // Lê os 4 bytes da memória (BRAM) para os registradores
                    reg_base   <= servo_memory[reg_addr][0];
                    reg_altura <= servo_memory[reg_addr][1];
                    reg_angulo <= servo_memory[reg_addr][2];
                    reg_garra  <= servo_memory[reg_addr][3];
                    state <= R_SEND_BASE; // Vai para o primeiro envio
                end
                
                R_SEND_BASE: begin
                    tx_dv <= 1'b1; // Ativa a transmissão
                    tx_byte <= reg_base; // Define o dado
                    next_state_on_tx_done <= R_SEND_ALTURA; // Próximo estado
                    state <= R_WAIT_TX_BUSY; // Aguarda o envio
                end
                
                R_SEND_ALTURA: begin
                    tx_dv <= 1'b1;
                    tx_byte <= reg_altura;
                    next_state_on_tx_done <= R_SEND_ANGULO;
                    state <= R_WAIT_TX_BUSY;
                end
                
                R_SEND_ANGULO: begin
                    tx_dv <= 1'b1;
                    tx_byte <= reg_angulo;
                    next_state_on_tx_done <= R_SEND_GARRA;
                    state <= R_WAIT_TX_BUSY;
                end
                
                R_SEND_GARRA: begin
                    tx_dv <= 1'b1;
                    tx_byte <= reg_garra;
                    next_state_on_tx_done <= IDLE; // Fim da sequência
                    state <= R_WAIT_TX_BUSY;
                end
                
                // Estado de espera (pausa a FSM enquanto o UART está ocupado)
                R_WAIT_TX_BUSY: begin
                    if (!tx_active) begin // tx_active vem do módulo uart_top
                        state <= next_state_on_tx_done; // Pula para o próximo estado
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
    // --- Lógica da Memória (BRAM) ---
    // Este bloco está sempre ativo e escreve na memória (BRAM)
    // no ciclo de clock exato em que 'save_trigger' é '1'.
    always_ff @(posedge clk) begin
        if (save_trigger) begin
            servo_memory[reg_addr][0] <= reg_base;
            servo_memory[reg_addr][1] <= reg_altura;
            servo_memory[reg_addr][2] <= reg_angulo;
            servo_memory[reg_addr][3] <= reg_garra;
        end
    end

    // --- Lógica dos 3 LEDs de Status ---
    
    // 1. LED de GRAVAÇÃO (L2) - Pisca 100ms
    localparam BLINK_CYCLES = clk_freq / 10; // 100ms
    logic [$clog2(BLINK_CYCLES):0] blink_counter;
    
    // 'led_grava_n' é ativo-baixo (acende com 0)
    // O LED fica APAGADO (1) a menos que o contador esteja ativo
    assign led_grava_n = (blink_counter == 0);
    
    // Lógica do contador (monoestável)
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin 
            blink_counter <= 0;
        end else begin
            if (save_trigger) begin // Inicia o pulso
                blink_counter <= BLINK_CYCLES; 
            end else if (blink_counter > 0) begin // Mantém o pulso
                blink_counter <= blink_counter - 1;
            end
        end
    end
    
    // 2. LED de ENVIO (C1) - Aceso durante o TX
    // 'tx_active' vem do módulo uart_top e é '1' quando o TX está ocupado.
    // O LED é ativo-baixo, então invertemos o sinal.
    assign led_envio_n = !tx_active;
    
    // 3. LED de REPRODUÇÃO (C2) - Aceso durante a Leitura
    // Lógica combinacional para verificar se a FSM está em *qualquer*
    // estado de leitura (R_...).
    logic is_reading_state;
    assign is_reading_state = (state == R_GET_ADDR) || (state == R_PREP_DATA) ||
                              (state == R_SEND_BASE) || (state == R_SEND_ALTURA) ||
                              (state == R_SEND_ANGULO) || (state == R_SEND_GARRA) ||
                              (state == R_WAIT_TX_BUSY);
                              
    // O LED é ativo-baixo, então invertemos o sinal.
    assign led_reproducao_n = !is_reading_state;

endmodule