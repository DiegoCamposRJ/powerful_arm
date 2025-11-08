# FPGA como Memória de Posições para Braço Robótico (Bitdoglab)
<p align="center">
  <img src="https://image-pro.easyeda.com/pullimages/022f49b9a9004b03bfb71f4710998550.webp" alt="Diagrama do Sistema" width="600" height="800">
</p>
Este projeto demonstra uma arquitetura de sistema embarcado onde uma FPGA Colorlight i9 (Lattice ECP5) atua como uma unidade de co-processamento e memória (BRAM) para um microcontrolador Bitdoglab (Raspberry Pi Pico).  
O objetivo é permitir que o Bitdoglab (o "cérebro") envie comandos para a FPGA (a "memória rápida") para Salvar ou Reproduzir sequências de movimentos de um braço robótico (coordenadas de 4 servos).  
A comunicação entre as duas placas é realizada via UART (a 115200 baud) usando um protocolo binário customizado.

## 🚀 Funcionalidades Principais

- **Armazenamento na FPGA**: Salva até 256 posições de servo (cada posição contendo 4 ângulos: Base, Altura, Ângulo e Garra) diretamente na BRAM da FPGA.
- **Comunicação via Protocolo**: Utiliza comandos binários simples para operações de Leitura (0xB0) e Escrita (0xA0).
- **Controle de Hardware Duplo**:
  - O Bitdoglab pode solicitar posições específicas da memória via UART (Comando 0xB0).
  - Um Botão Físico (E3) na FPGA pode ser pressionado para acionar a "reprodução" da Posição de Memória 0.
- **Feedback Visual**: A FPGA utiliza 3 LEDs de status (ativos-baixo) para diagnóstico visual em tempo real:
  - **LED D2 (led_grava_n)**: Pisca uma vez quando uma posição é salva com sucesso na memória.
  - **LED C1 (led_envio_n)**: Acende enquanto a FPGA está ativamente transmitindo dados (enviando) pela UART.
  - **LED C2 (led_reproducao_n)**: Acende durante toda a operação de leitura/reprodução de dados.
- **Toolchain Open Source**: O projeto da FPGA é construído usando Yosys, nextpnr-ecp5 e openFPGALoader.

## 🛠️ Hardware Utilizado

- **FPGA**: Colorlight i9 (Lattice ECP5 LFE5U-45F)
- **Microcontrolador**: Bitdoglab (Raspberry Pi Pico / RP2040)
- **Conexão**: 3 fios (GND Comum, TX, RX)

## 📡 Protocolo de Comunicação (UART @ 115200)

### 1. Salvar Posição (Bitdoglab → FPGA)
O Bitdoglab envia um pacote de 6 bytes:  
`[Comando 0xA0] [Endereço (0-255)] [Byte Base] [Byte Altura] [Byte Ângulo] [Byte Garra]`  

A FPGA recebe, salva na memória no Endereço especificado e pisca o LED `led_grava_n`.

### 2. Ler Posição (Bitdoglab → FPGA → Bitdoglab)
O Bitdoglab envia um pacote de 2 bytes:  
`[Comando 0xB0] [Endereço (0-255)]`  

A FPGA recebe, busca os dados na memória e responde (transmite) com um pacote de 4 bytes:  
`[Byte Base] [Byte Altura] [Byte Ângulo] [Byte Garra]`

### 3. Reprodução via Botão (FPGA → Bitdoglab)
O usuário pressiona o Botão T3 na FPGA.  
A FPGA busca os dados do Endereço 0 da memória e transmite o pacote de 4 bytes (Base, Altura, Ângulo, Garra) para o Bitdoglab.

## 📋 Como Usar

1. **Configuração da FPGA**:
   - Compile o projeto Verilog usando Yosys e nextpnr-ecp5.
   - Programe a FPGA com openFPGALoader.

2. **Conexão Física**:
   - Conecte GND, TX (FPGA → Bitdoglab) e RX (Bitdoglab → FPGA).

3. **Código no Bitdoglab**:
   - Implemente o envio de comandos UART conforme o protocolo descrito.
   - Exemplo básico (em MicroPython ou C++ para RP2040):  
     ```python
     import machine
     import time

     uart = machine.UART(0, baudrate=115200)

     # Salvar posição (exemplo: endereço 0, valores 128 para todos)
     cmd_save = b'\xA0\x00\x80\x80\x80\x80'
     uart.write(cmd_save)
     time.sleep(0.1)

     # Ler posição (exemplo: endereço 0)
     cmd_read = b'\xB0\x00'
     uart.write(cmd_read)
     response = uart.read(4)  # Aguarda 4 bytes
     print(response)
     ```

4. **Teste**:
   - Envie comandos via Bitdoglab e observe os LEDs na FPGA.
   - Pressione T3 para reprodução automática da posição 0.

## 🔧 Dependências e Ferramentas

- **FPGA**: Yosys, nextpnr-ecp5, openFPGALoader.
- **Bitdoglab**: MicroPython ou Pico SDK (RP2040).

## 📝 Contribuições

Sinta-se à vontade para abrir issues ou pull requests! Este projeto é open source e bem-vindo a melhorias.

## 📄 Licença

MIT License - veja o arquivo [LICENSE](LICENSE) para detalhes.

---

*Projeto desenvolvido por [@DiegoCamposRJ]. Contato: [https://orcid.org/0009-0008-4746-7296].*

*Projeto desenvolvido por [@cledilson-devcode]. Contato: [https://www.linkedin.com/in/cledilson-pinto-filho].*
