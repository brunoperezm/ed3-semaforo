/*
 * Hasta ahora tenemos en teoria la logica de los semaforos, y lo del ADC y lo EINT0 PARA EL CIEGO
 * */

#include "lpc17xx_adc.h"

#include "lpc17xx_uart.h"

#include "lpc17xx_exti.h"

#include "lpc17xx_gpio.h"

#include "lpc17xx_timer.h"

#include "lpc17xx_pinsel.h"

#include "LPC17xx.h"
#include "string.h"
#include "stdlib.h"
#include "limits.h"

#include <cr_section_macros.h>

#define BITN(x) (1 << x)
#define BUFFER_LIMIT 200
#define PWMPRESCALE (25 - 1) //25 PCLK cycles to increment TC by 1 i.e. 1 Micro-second

/*
            Sem 1     --  Sem 2
    Seq 0 -> Verde    --  Rojo
    Seq 1 -> Amarillo --  Amarillo
    Seq 2 -> Rojo     --  Verde
    Seq 3 -> Amarillo --  Amarillo
*/
uint8_t secuencia = 0;
uint8_t intencidad = 0; //que es el indice del vector intencidad
uint32_t secuencia_Tiempo_normal[4] = {
    1,
    1,
    1,
    1};
uint32_t secuencia_Tiempo_configurando[4] = {
    1,
    1,
    1,
    1};
uint32_t secuencia_Tiempo_ciego[4] = {
    15,
    5,
    15,
    10};

typedef enum
{
    ESTADO_NORMAL,
    ESTADO_TENUE,
    ESTADO_ROTO
} EstadoLeds;

struct Semaforo
{
    uint8_t puerto;

    uint8_t pin_rojo;

    uint8_t pin_amarillo;

    uint8_t pin_verde;
    EstadoLeds ledRojo;
    EstadoLeds ledVerde;
    EstadoLeds ledAmarillo;
} semaforoA, semaforoB;

TIM_MATCHCFG_Type MatchTCFG;

uint8_t receiveBuffer[BUFFER_LIMIT];

uint32_t intencion_cambio_rojo_verde = 0;
uint32_t intencion_cambio_amarillo = 0;
enum Intensidad
{
    NULA,
    MEDIA,
    ALTA
} intensidad;

typedef enum
{
    ERROR_COMANDO_NO_ENCONTRADO,
    COMANDO_CONFIGURACION,
    COMANDO_READ_CONFIG,
    COMANDO_MODIFICAR_TIEMPOS, // Los tiempos ingresados se devolverran en
    // intencion_cambio_rojo_verde y ntencion_cambio_amarillo
    ERROR_CANTIDAD_ARGUMENTOS_INCORRECTOS,
    ERROR_ARGUMENTO_NO_NUMERICO,
    ERROR_ARGUMENTO_FUERA_DE_RANGO,
    COMANDO_AMBULANCIA_1,
    COMANDO_AMBULANCIA_2,
    COMANDO_MODO_NORMAL,
    COMANDO_EXIT // Sale de configuracion y vuelve al modo anterior
} EntradaResult;

typedef enum
{
    NORMAL,        // Funciona normalmente,
                   //usando los tiempos de semaforo del arreglo secuencia_Tiempo_normal
    CONFIGURANDO,  // Está en modo configuración, amarillo titilando,
    AMBULANCIA_S1, // Deja en verde a semáforo S1
    AMBULANCIA_S2, // Deja en verde a semáforo S2,
    CIEGO          // Da prioridad al semaforo S1
} ModoFuncionamiento;

ModoFuncionamiento modo_funcionamiento = NORMAL;
ModoFuncionamiento prev_funcionamiento = NORMAL;

EntradaResult parseEntrada(uint8_t *entrada);
void printEntradaResult(EntradaResult result);
void handleComandoResult(EntradaResult result);

uint8_t receivedBufferPtr = BUFFER_LIMIT;

void confPin();
void apagarSemaforos(struct Semaforo *, struct Semaforo *);
void confTIMER0(void);
void confTIMER1(void);
void confADC();
void confUART();
void UART_IntReceive();
void UART_IntSend();
void initPWM(void);
void updatePulseWidth(uint8_t semNumber, enum Intensidad);

int main(void)
{

    semaforoA.puerto = 2;
    semaforoA.pin_rojo = 0;
    semaforoA.pin_amarillo = 1;
    semaforoA.pin_verde = 2;

    semaforoB.puerto = 2;
    semaforoB.pin_rojo = 3;
    semaforoB.pin_amarillo = 4;
    semaforoB.pin_verde = 5;
    intensidad = MEDIA;
    confPin();
    confTIMER0();
    ConfIntExt();
    confADC();
    confTIMER1();
    initPWM();
    confUart();
    enviarTextoInicial();
    while (1)
    {
    }
    return 0;
}

void confPin(struct Semaforo *semaforoA, struct Semaforo *semaforoB)
{
    // Configuro como salida el led que trae la placa
    GPIO_SetDir(0, 1 << 22, 1);

    /* LUCES:
     * P2.0-> ROJO DE SEMAFORO 1
     * P2.1-> AMARILLO DE SEMAFORO 1
     * P2.2-> VERDE DE SEMAFORO 1
     *
     * P2.3-> ROJO DE SEMAFORO 2
     * P2.4-> AMARILLO DE SEMAFORO 2
     * P2.5-> VERDE DE SEMAFORO 2
     */

    apagarSemaforos(&semaforoA, &semaforoB);
    /*---------------------- Configuramos pines para ADC ----------------------*/
    PINSEL_CFG_Type PinCfgADC;                   //creo la estructura del canal 0 del ADC
    PinCfgADC.Portnum = 0;                       //Puerto 0
    PinCfgADC.Pinnum = 23;                       //pin 23
    PinCfgADC.Funcnum = 1;                       //funcion 1 --> AD0.0 --> canal 0 del ADC
    PinCfgADC.Pinmode = PINSEL_PINMODE_TRISTATE; //ni pull up ni pull down
    PinCfgADC.OpenDrain = PINSEL_PINMODE_NORMAL; //No opendrain
    PINSEL_ConfigPin(&PinCfgADC);
    PinCfgADC.Pinnum = 24; //pin 24
    PINSEL_ConfigPin(&PinCfgADC);
    PinCfgADC.Pinnum = 25; //pin 25
    PINSEL_ConfigPin(&PinCfgADC);
    PinCfgADC.Pinnum = 26; //pin 25
    PINSEL_ConfigPin(&PinCfgADC);

    /*---------------------- Configuramos pin para EINT0 ----------------------*/

    PINSEL_CFG_Type PinCfg; //Pin de EINT0
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 10; //Puerto 2.10
    PinCfg.Portnum = 2;

    PINSEL_ConfigPin(&PinCfg);

    /* ------------ Configuramos pint para uart1 ------------ */
    PINSEL_CFG_Type PinUART1Cfg;
    //configuración pin de Tx y Rx
    PinUART1Cfg.Funcnum = 1;
    PinUART1Cfg.OpenDrain = 0;
    PinUART1Cfg.Pinmode = 0;
    PinUART1Cfg.Pinnum = 15;
    PinUART1Cfg.Portnum = 0;
    PINSEL_ConfigPin(&PinUART1Cfg);
    PinUART1Cfg.Pinnum = 16;
    PINSEL_ConfigPin(&PinUART1Cfg);

    LPC_PINCON->PINSEL4 |= (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10); //Select PWM1.1 output for Pin1.18
    return;
}
void ConfIntExt()
{

    EXTI_InitTypeDef exti_cfg; //Defino el nombre de la estructura

    exti_cfg.EXTI_Line = 0; // 0=enit0, 1=eint1, 2=enit2, 3=eint3.

    exti_cfg.EXTI_Mode = 1; // 0=sensible por nivel, 1 =sensible por flancos.

    exti_cfg.EXTI_polarity = 0; //0=sensible por bajo 1=sensible por alto pq es pull up externa

    EXTI_Config(&exti_cfg); //paso la configuración

    EXTI_ClearEXTIFlag(0); // limpia bandera

    NVIC_EnableIRQ(EINT0_IRQn); // habilitas interrupciones externas.
}

void confTIMER0(void)
{ //ESTE SE VA A USAR PARA LA LOGICA DE LOS SEMAFOROS
    TIM_TIMERCFG_Type TIMERCFG;

    TIMERCFG.PrescaleOption = TIM_PRESCALE_USVAL;

    TIMERCFG.PrescaleValue = 1000000;

    MatchTCFG.MatchChannel = 0;

    MatchTCFG.MatchValue = 5; // Lo pongo en cero para que matchee apenas comience

    MatchTCFG.ResetOnMatch = ENABLE;

    MatchTCFG.IntOnMatch = ENABLE;

    MatchTCFG.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

    MatchTCFG.StopOnMatch = DISABLE;

    TIM_ConfigMatch(LPC_TIM0, &MatchTCFG); //Configura Match0 en 1s.

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIMERCFG); //Inicializa el periferico.

    TIM_Cmd(LPC_TIM0, ENABLE); //Habilita el periferico.

    NVIC_EnableIRQ(TIMER0_IRQn); //Habilita interrupciones del periferico.

    return;
}

void TIMER0_IRQHandler(void)
{
    /* * LUCES:
     * P0.0-> ROJO DE SEMAFORO 1
     * P0.1-> AMARILLO DE SEMAFORO 1
     * P0.2-> VERDE DE SEMAFORO 1
     *
     * P0.3-> ROJO DE SEMAFORO 2
     * P0.4-> AMARILLO DE SEMAFORO 2
     * P0.5-> VERDE DE SEMAFORO 2
     */
    if (modo_funcionamiento == NORMAL ||
        modo_funcionamiento == CONFIGURANDO ||
        modo_funcionamiento == CIEGO)
    {
        apagarSemaforos(&semaforoA, &semaforoB);
    }
    switch (secuencia)
    {
    case 0:
        if (modo_funcionamiento == NORMAL || modo_funcionamiento == CIEGO)
        {
            updatePulseWidth(semaforoA.pin_rojo, intensidad);  //P0.0-> ROJO DE SEMAFORO 1
            updatePulseWidth(semaforoB.pin_verde, intensidad); //P0.5-> VERDE DE SEMAFORO 2
        }
        else if (modo_funcionamiento == CONFIGURANDO)
        {
            updatePulseWidth(semaforoA.pin_amarillo, intensidad);
            updatePulseWidth(semaforoB.pin_amarillo, intensidad);
        }
        break;

    case 1:
        if (modo_funcionamiento == NORMAL || modo_funcionamiento == CIEGO)
        {
            updatePulseWidth(semaforoA.pin_amarillo, intensidad);
            updatePulseWidth(semaforoB.pin_amarillo, intensidad);
        }
        else if (modo_funcionamiento == CONFIGURANDO)
        {
            // No hago nada
        }

        break;
    case 2:
        if (modo_funcionamiento == NORMAL || modo_funcionamiento == CIEGO)
        {
            updatePulseWidth(semaforoA.pin_verde, intensidad);
            updatePulseWidth(semaforoB.pin_rojo, intensidad);
        }
        else if (modo_funcionamiento == CONFIGURANDO)
        {
            updatePulseWidth(semaforoA.pin_amarillo, intensidad);
            updatePulseWidth(semaforoB.pin_amarillo, intensidad);
        }

        break;

    case 3:
        if (modo_funcionamiento == NORMAL || modo_funcionamiento == CIEGO)
        {
            updatePulseWidth(semaforoA.pin_amarillo, intensidad);
            updatePulseWidth(semaforoB.pin_amarillo, intensidad);
        }
        else if (modo_funcionamiento == CONFIGURANDO)
        {
            // No hago nada
        }

        break;
    }
    if (modo_funcionamiento == NORMAL)
    {
        TIM_UpdateMatchValue(LPC_TIM0, 0, secuencia_Tiempo_normal[secuencia]);
    }
    else if (modo_funcionamiento == CONFIGURANDO)
    {
        TIM_UpdateMatchValue(LPC_TIM0, 0, secuencia_Tiempo_configurando[secuencia]);
    }
    else if (modo_funcionamiento == CIEGO)
    {
        TIM_UpdateMatchValue(LPC_TIM0, 0, secuencia_Tiempo_ciego[secuencia]);
    }
    if (secuencia < 3)
    {
        secuencia++;
    }
    else
    {
        if (modo_funcionamiento == CIEGO)
        {
            modo_funcionamiento = NORMAL;
        }
        secuencia = 0; //VUELVE AL ESTADO 0  SEMAFORO 1: ROJO SEMAFORO 2: VERDE
    }

    TIM_ResetCounter(LPC_TIM0);
    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

void EINT0_IRQHandler(void)
{

    secuencia = 1; // Pone en amarillo para pasar verde al SEM 2
    modo_funcionamiento = CIEGO;
    TIMER0_IRQHandler();
    EXTI_ClearEXTIFlag(0); // limpia bandera
}
void confTIMER1(void)
{                                //ESTE VA A CONTROLAR CADA CUANTO MUESTREA EL ADC
    TIM_MATCHCFG_Type match_cfg; //creo la estructura del Match
    TIM_TIMERCFG_Type timer_cfg; //creo la estructura del timer

    //se puede calcular el timer para 10 min con Prescaler = 6000000, en valor absoluto, no en micro segundos y 10000 el match

    timer_cfg.PrescaleOption = TIM_PRESCALE_USVAL; //1: valor de preescaler en microsegundos (USVAL)
    timer_cfg.PrescaleValue = 100;                 //valor de preescaler dependiendo el modo, en este caso 6 millones de us =6 segundos, supongo que quiere decir que cada 6 segundos se va a incrementar en 1 el timer
    /*---------------------------------------------------------------------------------------------------------------*/
    match_cfg.MatchChannel = 0;                          //configuro el MATCH0 que se va a habilitar
    match_cfg.IntOnMatch = ENABLE;                       //cuando matchee que interrumpa el timer.
    match_cfg.StopOnMatch = DISABLE;                     //cuando matchee que no detenga el timer.
    match_cfg.ResetOnMatch = ENABLE;                     //cuando matchee que resetee el timer.
    match_cfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING; //no hago nada con el pin de match externo, si no lo tendría que configurar con el pinsel.
    //REVISAR CALCULO
    match_cfg.MatchValue = 10000 * 1; //cuando el timer llegue a 100 match es decir 600 segundos o 10 minutos.
    /*---------------------------------------------------------------------------------------------------------------*/
    TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &timer_cfg); //Cargo configuracion de TIMER: que timer uso- modo del timer - a que configuracion del timer hago referencia
    TIM_ConfigMatch(LPC_TIM1, &match_cfg);          // Cargo configuracion de MATCH que defini
    TIM_Cmd(LPC_TIM1, ENABLE);                      // activa timer1
    NVIC_EnableIRQ(TIMER1_IRQn);
    return;
}

void TIMER1_IRQHandler(void)
{

    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
}

void confADC()
{

    ADC_Init(LPC_ADC, 2500);                        //FRECUENCIA DE TRABAJO del ADC. determinar cuando tiene que convertir el ADC
    ADC_BurstCmd(LPC_ADC, 0);                       //Modo START, NO burst
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE); //activación de canal 0
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, SET);      //activo interrupción para canal 0
    // Va a usar el P0.23 como entrada del ADC

    NVIC_EnableIRQ(ADC_IRQn); //habilitamos interrupciones por adc
}

void ADC_IRQHandler(void)
{ //cada vez que convierta vamos a decidir cuantos leds de cada luz se deben encender

    uint8_t canal_que_interrumpio = ((LPC_ADC->ADGDR) >> 24) & 0b111;
    uint8_t proximo_canal = 0;
    uint16_t adc_value = 0; //variable para alamcenar el valor de la conversion del ADC
    switch (canal_que_interrumpio)
    {
    case 0: // Interrumpio canal 0
        adc_value = ((LPC_ADC->ADDR0) >> 4) & 0XFFF;
        proximo_canal = 0b10; // Canal 1
        break;
    case 1: // Interrumpio canal 1
        adc_value = ((LPC_ADC->ADDR1) >> 4) & 0XFFF;
        proximo_canal = 0b100; // Canal 2
        break;
    case 2: // Interrumpio canal 2
        adc_value = ((LPC_ADC->ADDR2) >> 4) & 0XFFF;
        proximo_canal = 0b1000; // Canal 3
        break;
    case 3: // Interrumpio canal 3
        adc_value = ((LPC_ADC->ADDR3) >> 4) & 0XFFF;
        proximo_canal = 0b1; // Canal 0
        break;

    default:
        proximo_canal = 0b1; //
        break;
    }

    LPC_ADC->ADCR &= ~(0x7F);       // Pongo en cero el AD0CR[0:7] para borrar los canales activados
    LPC_ADC->ADCR |= proximo_canal; // Pongo en cero el AD0CR[0:7] para borrar los canales activados

    // Obtengo el nuevo estado
    EstadoLeds estadoLed;
    if (adc_value < 4096 && adc_value > 2500)
    {
        estadoLed = ESTADO_NORMAL;
    }
    else if (adc_value <= 2500 && adc_value > 500)
    {
        estadoLed = ESTADO_TENUE;
    }
    else
    {
        estadoLed = ESTADO_ROTO;
    }

    // Canal 0, osea LED VERDE
    if (canal_que_interrumpio == 0)
    {
        semaforoA.ledVerde = estadoLed;
    }
    // Canal 1, osea LED ROJO
    else if (canal_que_interrumpio == 1)
    {
        semaforoA.ledRojo = estadoLed;
    }
    // Canal 2, osea LED AMARILLO
    else if (canal_que_interrumpio == 2)
    {
        semaforoA.ledAmarillo = estadoLed;
    }
    // Canal 3, el que mide intensidad
    else if (canal_que_interrumpio == 3)
    {
        enum Intensidad intensidadMedida;
        if (adc_value > 2000)
        {
            intensidadMedida = ALTA;
        }
        else
        {
            intensidadMedida = MEDIA;
        }
        intensidad = intensidadMedida;
    }
}

void apagarSemaforos(struct Semaforo *semaforoA, struct Semaforo *semaforoB)
{
    LPC_PWM1->MR1 = 0;
    LPC_PWM1->MR2 = 0;
    LPC_PWM1->MR3 = 0;
    LPC_PWM1->MR4 = 0;
    LPC_PWM1->MR5 = 0;
    LPC_PWM1->MR6 = 0;
    LPC_PWM1->LER |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6); //Load the MR1 new value at start of next cycle
}

void confUart(void)
{
    UART_CFG_Type UARTConfigStruct;
    UART_FIFO_CFG_Type UARTFIFOConfigStruct;
    //configuración por defecto:
    UART_ConfigStructInit(&UARTConfigStruct);
    //inicializa periférico
    UART_Init(LPC_UART1, &UARTConfigStruct);
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
    //Inicializa FIFO
    UART_FIFOConfig(LPC_UART1, &UARTFIFOConfigStruct);
    //Habilita transmisión
    UART_TxCmd(LPC_UART1, ENABLE);
    // Habilita interrupción por el RX del UART
    UART_IntConfig(LPC_UART1, UART_INTCFG_RBR, ENABLE);
    // Habilita interrupción por el estado de la linea UART
    UART_IntConfig(LPC_UART1, UART_INTCFG_RLS, ENABLE);
    //NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
    //Habilita interrupción por UART1
    NVIC_EnableIRQ(UART1_IRQn);
    return;
}

void UART1_IRQHandler(void)
{
    uint32_t intsrc, tmp, tmp1;
    //Determina la fuente de interrupción
    intsrc = UART_GetIntId(LPC_UART1);
    tmp = intsrc & UART_IIR_INTID_MASK;
    // Evalúa Line Status
    if (tmp == UART_IIR_INTID_RLS)
    {
        tmp1 = UART_GetLineStatus(LPC_UART1);
        tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
        // ingresa a un Loop infinito si hay error
        if (tmp1)
        {
            // Hay un error, por ahora no hacemos nada
        }
    }
    // Receive Data Available or Character time-out
    if ((tmp == UART_IIR_INTID_RDA))
    {
        UART_IntReceive();
    }
    return;
}

void UART_IntReceive()
{
    uint8_t info[1] = "";
    UART_Receive(LPC_UART1, info, sizeof(info), NONE_BLOCKING);

    if (receivedBufferPtr == BUFFER_LIMIT)
    {
        receivedBufferPtr = 0;
    }
    if (info[0] == '\r')
    {
        // El usuario mando un "enter"
        info[0] = '\0';
        receiveBuffer[receivedBufferPtr] = info[0];
        receivedBufferPtr = BUFFER_LIMIT;                   // Resteo el buffer
        EntradaResult result = parseEntrada(receiveBuffer); // Interpretación de entrada
        handleComandoResult(result);                        // Acción de la entrada
    }
    else
    {
        receiveBuffer[receivedBufferPtr] = info[0];
    }

    if (receivedBufferPtr >= BUFFER_LIMIT)
    {
        receivedBufferPtr = BUFFER_LIMIT;
    }
    else
    {
        receivedBufferPtr++;
    }
}

/*
 * Interpreta la entrada y devuelve un EntradaResult
 */

EntradaResult parseEntrada(uint8_t *entrada)
{
    // Veo si es "conf"
    if (strcmp(entrada, "conf") == 0)
    {
        return COMANDO_CONFIGURACION;
    }

    // Veo si es {sem} {tiempo rojo/verde} {tiempo amarillo}
    if (entrada[0] == 's' && entrada[1] == 'e' && entrada[2] == 'm')
    {
        char buffer[BUFFER_LIMIT];
        strcpy(buffer, entrada);

        // strtok divide la cadena con el delimitador " "
        uint8_t *token = strtok(buffer, " ");
        uint8_t *endptr;
        uint8_t cont = 0;
        uint32_t rojo_verde;
        uint32_t amarillo;

        // El primer token va a ser "s1" así que ejecuto una vez mas strtok
        cont++;
        token = strtok(NULL, " ");

        while (token != NULL)
        {

            // strtol es para transformar una cadena a un número
            int32_t val = strtol(token, &endptr, 10);

            if (endptr == token || *endptr != '\0')
            {
                return ERROR_ARGUMENTO_NO_NUMERICO;
            }
            if (val <= 0 || val == LONG_MAX || val == LONG_MIN)
            {
                return ERROR_ARGUMENTO_FUERA_DE_RANGO;
            }

            switch (cont)
            {
            case 0: // es el caso del primer token "s1", no hago nada
                break;
            case 1:
                intencion_cambio_rojo_verde = val;
                break;
            case 2:
                intencion_cambio_amarillo = val;
                break;

            default: // Son 2 tokens después de s1
                return ERROR_CANTIDAD_ARGUMENTOS_INCORRECTOS;
            }
            cont++;
            token = strtok(NULL, " ");
        }
        if (cont != 3)
        { // Hay 3 argumentos contando el "sem"
            return ERROR_CANTIDAD_ARGUMENTOS_INCORRECTOS;
        }
        else
        {
            return COMANDO_MODIFICAR_TIEMPOS;
        }
    }
    if (strcmp(entrada, "read_config") == 0)
    {
        return COMANDO_READ_CONFIG;
    }
    if (strcmp(entrada, "ambulancia s1") == 0)
    {
        return COMANDO_AMBULANCIA_1;
    }
    if (strcmp(entrada, "ambulancia s2") == 0)
    {
        return COMANDO_AMBULANCIA_2;
    }
    if (strcmp(entrada, "modo_normal") == 0)
    {
        return COMANDO_MODO_NORMAL;
    }
    if (strcmp(entrada, "exit") == 0)
    {
        return COMANDO_EXIT;
    }
    return ERROR_COMANDO_NO_ENCONTRADO;
}

void printEntradaResult(EntradaResult result)
{
    uint8_t *msg;
    switch (result)
    {
    case ERROR_COMANDO_NO_ENCONTRADO:
        msg = "Comando  no encontrado\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case ERROR_CANTIDAD_ARGUMENTOS_INCORRECTOS:
        msg = "Cantidad de argumentos ingresados invalidos\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case ERROR_ARGUMENTO_NO_NUMERICO:
        msg = "Argumento ingresado no numerico\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case ERROR_ARGUMENTO_FUERA_DE_RANGO:
        msg = "Argumento ingresado fuera de rango\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case COMANDO_CONFIGURACION:
        msg = "- Entro en modo configuracion\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case COMANDO_MODO_NORMAL:
        msg = "- Entro en modo normal\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case COMANDO_READ_CONFIG:
        msg = "- Leer configuracion\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case COMANDO_MODIFICAR_TIEMPOS:
        msg = "- Modificar tiempos\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case COMANDO_AMBULANCIA_1:
        msg = "- Ambulancia Semaforo 1\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case COMANDO_AMBULANCIA_2:
        msg = "- Ambulancia Semaforo 2\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    case COMANDO_EXIT:
        msg = "- Exit, vuelve a modo anterior\r\n";
        UART_Send(LPC_UART1, msg, strlen(msg), BLOCKING);
        break;
    default:
        break;
    }
}

void handleComandoResult(EntradaResult result)
{
    if (result != COMANDO_CONFIGURACION && modo_funcionamiento != CONFIGURANDO)
    {
        uint8_t *text;
        text = "No esta en modo configuracion\n\rPara ello ingrese $conf\n\r";
        UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
        return;
    }
    if (result == COMANDO_CONFIGURACION)
    {
        prev_funcionamiento = modo_funcionamiento;
    }
    printEntradaResult(result);
    switch (result)
    {
    case ERROR_COMANDO_NO_ENCONTRADO:
    case ERROR_ARGUMENTO_NO_NUMERICO:
    case ERROR_ARGUMENTO_FUERA_DE_RANGO:
    case ERROR_CANTIDAD_ARGUMENTOS_INCORRECTOS:
        break;
    case COMANDO_CONFIGURACION:
        modo_funcionamiento = CONFIGURANDO;
        TIMER0_IRQHandler();
        break;
    case COMANDO_MODO_NORMAL:
        modo_funcionamiento = NORMAL;
        TIMER0_IRQHandler();
        break;
    case COMANDO_READ_CONFIG:
        print_current_config(modo_funcionamiento);
        break;
    case COMANDO_MODIFICAR_TIEMPOS:
        secuencia_Tiempo_normal[1] = intencion_cambio_amarillo;
        secuencia_Tiempo_normal[3] = intencion_cambio_amarillo;
        secuencia_Tiempo_normal[0] = intencion_cambio_rojo_verde;
        secuencia_Tiempo_normal[2] = intencion_cambio_rojo_verde;
        modo_funcionamiento = NORMAL;
        break;
    case COMANDO_AMBULANCIA_1:
        modo_funcionamiento = AMBULANCIA_S1;
        apagarSemaforos(&semaforoA, &semaforoB);
        updatePulseWidth(semaforoA.pin_verde, intensidad);
        updatePulseWidth(semaforoB.pin_rojo, intensidad);
        break;
    case COMANDO_AMBULANCIA_2:
        apagarSemaforos(&semaforoA, &semaforoB);
        updatePulseWidth(semaforoB.pin_verde, intensidad);
        updatePulseWidth(semaforoA.pin_rojo, intensidad);
        modo_funcionamiento = AMBULANCIA_S2;
        break;
    case COMANDO_EXIT:
        modo_funcionamiento = prev_funcionamiento;
        TIMER0_IRQHandler();
        break;
    }
}
/**
 * Envía un texto descriptivo de bienvenida con un explicativo de los
 * posibles comandos que se pueden usar para configurar
 * el proyecto
*/
void enviarTextoInicial()
{
    uint8_t inicial[] =
        "Bienviendo a Semaforo 1.0\n\r"
        "Para entrar en modo configuracion ingrese\n\r"
        "$ conf\n\r"
        "Comandos:\n\r"
        "$ sem {tiempo rojo/verde} {tiempo amarillo}\n\r"
        "$ read_config\n\r"
        "$ modo_normal\n\r"
        "$ ambulancia {s1|s2}\n\r"
        "Para volver al modo anterior y salir de configuracion\n\r"
        "$ exit\n\r";
    UART_Send(LPC_UART1, inicial, sizeof(inicial), BLOCKING);
}

void print_current_config()
{
    uint8_t *text;
    text = "Tiempo semaforo amarillo: ";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    uint8_t buffer[10];
    if (prev_funcionamiento == CIEGO)
    {
        itoa(secuencia_Tiempo_ciego[1], buffer, 10);
    }
    else
    {
        itoa(secuencia_Tiempo_normal[1], buffer, 10);
    }
    UART_Send(LPC_UART1, buffer, strlen(buffer), BLOCKING);
    uint8_t buffer1[10];
    if (prev_funcionamiento == CIEGO)
    {
        itoa(secuencia_Tiempo_ciego[2], buffer1, 10);
    }
    else
    {
        itoa(secuencia_Tiempo_normal[2], buffer1, 10);
    }

    text = "\n\rTiempo semaforo rojo/verde: ";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    UART_Send(LPC_UART1, buffer1, strlen(buffer1), BLOCKING);
    text = "\n\r";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    text = "Modo de funcionamiento: ";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    switch (prev_funcionamiento)
    {
    case NORMAL:
        text = "MODO NORMAL";
        break;
    case CONFIGURANDO:
        text = "CONFIGURANDO";
        break;
    case AMBULANCIA_S1:
        text = "AMBULANCIA S1";
        break;
    case AMBULANCIA_S2:
        text = "AMBULANCIA S2";
        break;
    case CIEGO:
        text = "PRIORIDAD CIEGO";
        break;
    }
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    text = "\n\r";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);

    text = "Estado Leds Semaforo A:\n\r";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    text = "Verde: ";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    switch (semaforoA.ledVerde)
    {
    case ESTADO_NORMAL:
        text = "ESTADO NORMAL";
        break;
    case ESTADO_TENUE:
        text = "TENUE: posiblemente requiera mantenimiento";
        break;
    case ESTADO_ROTO:
        text = "ROTO: luz no se ve, requiere cambio inmediato";
        break;
    }
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    text = "\n\r";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);

    text = "Amarillo: ";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    switch (semaforoA.ledAmarillo)
    {
    case ESTADO_NORMAL:
        text = "ESTADO NORMAL";
        break;
    case ESTADO_TENUE:
        text = "TENUE: posiblemente requiera mantenimiento";
        break;
    case ESTADO_ROTO:
        text = "ROTO: luz no se ve, requiere cambio inmediato";
        break;
    }
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    text = "\n\r";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);

    text = "Rojo: ";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    switch (semaforoA.ledRojo)
    {
    case ESTADO_NORMAL:
        text = "ESTADO NORMAL";
        break;
    case ESTADO_TENUE:
        text = "TENUE: posiblemente requiera mantenimiento";
        break;
    case ESTADO_ROTO:
        text = "ROTO: luz no se ve, requiere cambio inmediato";
        break;
    }
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
    text = "\n\r";
    UART_Send(LPC_UART1, text, strlen(text), BLOCKING);
}
void initPWM(void)
{
    /*Assuming that PLL0 has been setup with CCLK = 100Mhz and PCLK = 25Mhz.*/

    //LPC_PINCON->PINSEL4 |= (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6) | (1 << 8) | (1 << 10); //Select PWM1.1 output for Pin1.18
    LPC_SC->PCONP |= (1 << 6);  // PWM on
                                //LPC_PWM1->PCR = 0x0;                                                                     //Select Single Edge PWM - by default its single Edged so this line can be removed
    LPC_PWM1->PR = PWMPRESCALE; //1 micro-second resolution
    LPC_PWM1->MR0 = 1000;       //1000us = 1ms period duration
    LPC_PWM1->MR1 = 15;         //250us - default pulse duration i.e. width
    LPC_PWM1->MR2 = 200;
    LPC_PWM1->MR3 = 100;
    LPC_PWM1->MR4 = 15;
    LPC_PWM1->MR5 = 200;
    LPC_PWM1->MR6 = 100;
    LPC_PWM1->MCR = (1 << 1);                                                                    //Reset PWM TC on PWM1MR0 match
    LPC_PWM1->LER |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6); //update values in MR0 and MR1
    LPC_PWM1->PCR |= (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14);       //enable PWM output
    LPC_PWM1->TCR = (1 << 1);                                                                    //Reset PWM TC & PR

    LPC_PWM1->TCR = (1 << 0) | (1 << 3); //enable counters and PWM Mode
}

void updatePulseWidth(uint8_t semNumber, enum Intensidad intensidad)
{
    if (GPIO_ReadValue(0) & 1 << 22)
    {
        GPIO_ClearValue(0, 1 << 22);
    }
    else
    {
        GPIO_SetValue(0, 1 << 22);
    }
    uint16_t duty = 0;
    switch (semNumber)
    {
    case 0: // Rojo, uso valores bajos porque se ve fuerte
        if (intensidad == NULA)
            duty = 0;
        else if (intensidad == MEDIA)
            duty = 15;
        else
            duty = 80;
        LPC_PWM1->MR1 = duty;
        break;
    case 1: // Amarillo valores bien altos
        if (intensidad == NULA)
            duty = 0;
        else if (intensidad == MEDIA)
            duty = 200;
        else
            duty = 1000;
        LPC_PWM1->MR2 = duty;
        break;
    case 2: // Verde valores medios
        if (intensidad == NULA)
            duty = 0;
        else if (intensidad == MEDIA)
            duty = 100;
        else
            duty = 500;
        LPC_PWM1->MR3 = duty;
        break;
    case 3:
        if (intensidad == NULA)
            duty = 0;
        else if (intensidad == MEDIA)
            duty = 15;
        else
            duty = 80;
        LPC_PWM1->MR4 = duty;
        break;
    case 4:
        if (intensidad == NULA)
            duty = 0;
        else if (intensidad == MEDIA)
            duty = 200;
        else
            duty = 1000;
        LPC_PWM1->MR5 = duty;
        break;
    case 5:
        if (intensidad == NULA)
            duty = 0;
        else if (intensidad == MEDIA)
            duty = 100;
        else
            duty = 500;
        LPC_PWM1->MR6 = duty;
        break;

    default:
        break;
    }
    LPC_PWM1->LER |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6); //Load the MR1 new value at start of next cycle
}
