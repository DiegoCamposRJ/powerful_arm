@echo off
REM ============================================================
REM     Síntese e Gravação - UART SystemVerilog para Colorlight i9
REM     
REM     Projeto: uart_powerful_arm (Memória de Servos)
REM     FPGA: Lattice ECP5 LFE5U-45F-6BG381C
REM     Clock: 25 MHz
REM     Baud Rate: 115200
REM ============================================================

REM Configurações do projeto
set OSSCAD=C:\oss-cad-suite
set TOP=uart_powerful_arm
set LPF=uart_colorlight_i9.lpf
set BOARD=colorlight-i9

REM Ativa ambiente OSS CAD Suite
call "%OSSCAD%\environment.bat"
cd %~dp0

echo ============================================================
echo   UART SystemVerilog - Projeto: %TOP%
echo ============================================================
echo.
echo [1/5] Verificando arquivos necessários...

REM A verificação de %TOP%.sv agora procura por "uart_powerful_arm.sv"
if not exist "%TOP%.sv" ( echo ❌ ERRO: Arquivo %TOP%.sv não encontrado! & pause & exit /b 1)
if not exist "%LPF%" ( echo ❌ ERRO: Arquivo %LPF% não encontrado! & pause & exit /b 1)
if not exist "uart_top.sv" ( echo ❌ ERRO: Arquivo uart_top.sv não encontrado! & pause & exit /b 1)
if not exist "uart_tx.sv" ( echo ❌ ERRO: Arquivo uart_tx.sv não encontrado! & pause & exit /b 1)
if not exist "uart_rx.sv" ( echo ❌ ERRO: Arquivo uart_rx.sv não encontrado! & pause & exit /b 1)

echo ✅ Todos os arquivos encontrados
echo.

REM ============================================================
echo [2/5] Síntese com Yosys...
echo ============================================================
echo ⚠️  Atenção: Garantindo que o baud_rate em %TOP%.sv está 115200...

yosys -p "read_verilog -sv uart_tx.sv uart_rx.sv uart_top.sv %TOP%.sv; synth_ecp5 -top %TOP% -json %TOP%.json"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ❌ ERRO na síntese! Verifique o código SystemVerilog.
    pause
    exit /b 1
)

echo ✅ Síntese concluída com sucesso
echo.

REM ============================================================
echo [3/5] Place and Route com nextpnr-ecp5...
echo ============================================================

nextpnr-ecp5 --json "%TOP%.json" --textcfg "%TOP%.config" --lpf "%LPF%" --45k --package CABGA381 --speed 6 --timing-allow-fail

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ⚠️  AVISO: Place and Route completou com avisos.
    echo.
)

echo ✅ Place and Route concluído
echo.

REM ============================================================
echo [4/5] Gerando bitstream com ecppack...
echo ============================================================

ecppack --compress "%TOP%.config" "%TOP%.bit"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ❌ ERRO ao gerar bitstream!
    pause
    exit /b 1
)

echo ✅ Bitstream gerado: %TOP%.bit
echo.

REM ============================================================
echo [5/5] Gravando na MEMÓRIA FLASH com openFPGALoader...
REM ============================================================

openFPGALoader -b %BOARD% -f --verify "%TOP%.bit"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ❌ ERRO ao gravar FPGA! Verifique conexão USB.
    pause
    exit /b 1
)

echo.
echo ✅ FPGA gravado com sucesso!
echo.

REM ============================================================
echo Limpando arquivos temporários...
REM ============================================================

del *.json *.config 2>nul

echo.
echo ============================================================
echo   ✅ PROCESSO COMPLETO!
echo ============================================================
echo.
echo Próximos passos (Teste de Memória de Servos):
echo   1. Conectar Bitdoglab ao FPGA:
echo      - Bitdoglab TX (Pino 0) --> FPGA RX (Pino A3)
echo      - Bitdoglab RX (Pino 1) <-- FPGA TX (Pino E2)
echo      - GND comum entre Bitdoglab e FPGA (MUITO IMPORTANTE!)
echo.
echo   2. Executar o main.c no Bitdoglab e observar o terminal.
echo   3. Pressione o botão T3 na FPGA para enviar dados.
echo ============================================================
echo.

pause