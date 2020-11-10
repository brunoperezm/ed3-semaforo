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

#include <cr_section_macros.h>


#define BITN(x)(1 << x)
#define BUFFER_LIMIT 200

/*
            Sem 1     --  Sem 2
    Seq 0 -> Verde    --  Rojo
    Seq 1 -> Amarillo --  Amarillo
    Seq 2 -> Rojo     --  Verde
    Seq 3 -> Amarillo --  Amarillo
*/
uint8_t secuencia = 0;
uint8_t intencidad = 0; //que es el indice del vector intencidad
uint8_t modo = 0; //El modo es 0 para normal, 1 para ciego y 2 para uart
uint32_t secuencia_Tiempo[4] = {
    10,
    5,
    10,
    5
};
uint32_t Intensidad[4] = {
    1000,
    2000,
    4000,
    5000
};

struct Semaforo {
    uint8_t puerto;

    uint8_t pin_rojo;

    uint8_t pin_amarillo;

    uint8_t pin_verde;
}
semaforoA, semaforoB;

TIM_MATCHCFG_Type MatchTCFG;

uint8_t sendBuffer[BUFFER_LIMIT];
uint8_t receiveBuffer[BUFFER_LIMIT];

/*
    sentBufferPtr Indica dónde esta el inicio del próximo paquete a transmitir en sendBuffer
    Ejemplo: sentBufferPtr[20] indica q se debe transmitir desde sendBuffer[20] hasta sendBuffer[35]
    sentBufferPtr = 0 indica que ya se puede transmitir
    sentBufferPtr = BUFFER_LIMIT indica que no hay nada que transmitir
*/
uint8_t sentBufferPtr = BUFFER_LIMIT;
uint8_t receivedBufferPtr = BUFFER_LIMIT;


void confPin(struct Semaforo * , struct Semaforo * );
void apagarSemaforos(struct Semaforo * , struct Semaforo * );
void confTIMER0(void);
void confTIMER1(void);
void confADC();
void confUART();
void UART_IntReceive();
void UART_IntSend();

int main(void) {

    semaforoA.puerto = 2;
    semaforoA.pin_rojo = 2;
    semaforoA.pin_amarillo = 3;
    semaforoA.pin_verde = 4;

    semaforoB.puerto = 2;
    semaforoB.pin_rojo = 5;
    semaforoB.pin_amarillo = 6;
    semaforoB.pin_verde = 7;

    confPin( & semaforoA, & semaforoB);
    confTIMER0();
    ConfIntExt();
    confTIMER1();
    confADC();
    confUart();
    uint8_t inicial[] = "TP3-FCEFyN\n\r";
    UART_Send(LPC_UART1, inicial, sizeof(inicial), NONE_BLOCKING);
    strcpy(sendBuffer,"Hola este es un ejemplo de una linea de texto muy larga. \n\r bla blabla \n\r sdesfsdf");
    sentBufferPtr = 0;
    while (1) {

    }
    return 0;
}

void confPin(struct Semaforo * semaforoA, struct Semaforo * semaforoB) {

    /* LUCES:
     * P2.2-> ROJO DE SEMAFORO 1
     * P2.3-> AMARILLO DE SEMAFORO 1
     * P2.4-> VERDE DE SEMAFORO 1
     *
     * P2.5-> ROJO DE SEMAFORO 2
     * P2.6-> AMARILLO DE SEMAFORO 2
     * P2.7-> VERDE DE SEMAFORO 2
     */
    // 6 SALIDAS LEDS
    uint32_t confSemaforoA = 0x0;
    uint32_t confSemaforoB = 0x0;

    confSemaforoA |= BITN(semaforoA -> pin_rojo);
    confSemaforoA |= BITN(semaforoA -> pin_amarillo);
    confSemaforoA |= BITN(semaforoA -> pin_verde);
    if (semaforoA -> puerto == semaforoB -> puerto) {
        confSemaforoB = confSemaforoA;
        confSemaforoB |= BITN(semaforoB -> pin_rojo);
        confSemaforoB |= BITN(semaforoB -> pin_amarillo);
        confSemaforoB |= BITN(semaforoB -> pin_verde);
        GPIO_SetDir(semaforoA -> puerto, confSemaforoB, 1);
        GPIO_ClearValue(semaforoA -> puerto, confSemaforoA);
    } else {
        confSemaforoB |= BITN(semaforoB -> pin_rojo);
        confSemaforoB |= BITN(semaforoB -> pin_amarillo);
        confSemaforoB |= BITN(semaforoB -> pin_verde);
        GPIO_SetDir(semaforoA -> puerto, confSemaforoA, 1);
        GPIO_SetDir(semaforoB -> puerto, confSemaforoB, 1);
        GPIO_ClearValue(semaforoA -> puerto, confSemaforoA);
        GPIO_ClearValue(semaforoB -> puerto, confSemaforoB);
    }

    /*---------------------- Configuramos pines para ADC ----------------------*/
    PINSEL_CFG_Type PinCfgADC; //creo la estructura del canal 0 del ADC
    PinCfgADC.Portnum = 0; //Puerto 0
    PinCfgADC.Pinnum = 23; //pin 23
    PinCfgADC.Funcnum = 1; //funcion 1 --> AD0.0 --> canal 0 del ADC
    PinCfgADC.Pinmode = PINSEL_PINMODE_TRISTATE; //ni pull up ni pull down
    PinCfgADC.OpenDrain = PINSEL_PINMODE_NORMAL; //No opendrain
    PINSEL_ConfigPin( & PinCfgADC);

    /*---------------------- Configuramos pin para EINT0 ----------------------*/

    PINSEL_CFG_Type PinCfg; //Pin de EINT0
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 10; //Puerto 2.10
    PinCfg.Portnum = 2;

    PINSEL_ConfigPin( & PinCfg);


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

    return;
}
void ConfIntExt() {

    EXTI_InitTypeDef exti_cfg; //Defino el nombre de la estructura

    exti_cfg.EXTI_Line = 0; // 0=enit0, 1=eint1, 2=enit2, 3=eint3.

    exti_cfg.EXTI_Mode = 1; // 0=sensible por nivel, 1 =sensible por flancos.

    exti_cfg.EXTI_polarity = 0; //0=sensible por bajo 1=sensible por alto pq es pull up externa

    EXTI_Config( & exti_cfg); //paso la configuración

    EXTI_ClearEXTIFlag(0); // limpia bandera

    NVIC_EnableIRQ(EINT0_IRQn); // habilitas interrupciones externas.

}

void confTIMER0(void) { //ESTE SE VA A USAR PARA LA LOGICA DE LOS SEMAFOROS
    TIM_TIMERCFG_Type TIMERCFG;

    TIMERCFG.PrescaleOption = TIM_PRESCALE_USVAL;

    TIMERCFG.PrescaleValue = 1000000;

    MatchTCFG.MatchChannel = 0;

    MatchTCFG.MatchValue = 5; // Lo pongo en cero para que matchee apenas comience

    MatchTCFG.ResetOnMatch = ENABLE;

    MatchTCFG.IntOnMatch = ENABLE;

    MatchTCFG.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;

    MatchTCFG.StopOnMatch = DISABLE;

    TIM_ConfigMatch(LPC_TIM0, & MatchTCFG); //Configura Match0 en 1s.

    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, & TIMERCFG); //Inicializa el periferico.

    TIM_Cmd(LPC_TIM0, ENABLE); //Habilita el periferico.

    NVIC_EnableIRQ(TIMER0_IRQn); //Habilita interrupciones del periferico.

    return;
}

void TIMER0_IRQHandler(void) {
    /* * LUCES:
     * P0.0-> ROJO DE SEMAFORO 1
     * P0.1-> AMARILLO DE SEMAFORO 1
     * P0.2-> VERDE DE SEMAFORO 1
     *
     * P0.3-> ROJO DE SEMAFORO 2
     * P0.4-> AMARILLO DE SEMAFORO 2
     * P0.5-> VERDE DE SEMAFORO 2
     */
    apagarSemaforos( & semaforoA, & semaforoB);
    switch (secuencia) {
    case 0:
        if (modo == 0) {

            GPIO_SetValue(semaforoA.puerto, BITN(semaforoA.pin_rojo)); //P0.0-> ROJO DE SEMAFORO 1
            GPIO_SetValue(semaforoB.puerto, BITN(semaforoB.pin_verde)); //P0.5-> VERDE DE SEMAFORO 2

            //MatchTCFG.MatchValue=secuencia_Tiempo[1];

        }

        break;

    case 1:
        if (modo == 0) {

            GPIO_SetValue(semaforoA.puerto, BITN(semaforoA.pin_amarillo)); // P0.1-> AMARILLO DE SEMAFORO 1
            GPIO_SetValue(semaforoB.puerto, BITN(semaforoB.pin_amarillo)); // P0.4-> AMARILLO DE SEMAFORO 2

        }

        break;
    case 2:
        if (modo == 0) {

            GPIO_SetValue(semaforoA.puerto, BITN(semaforoA.pin_verde)); //P0.2-> VERDE DE SEMAFORO 1
            GPIO_SetValue(semaforoB.puerto, BITN(semaforoB.pin_rojo)); //P0.3-> ROJO DE SEMAFORO 2

        }

        break;

    case 3:
        if (modo == 0) {

            GPIO_SetValue(semaforoA.puerto, BITN(semaforoA.pin_amarillo)); //P0.1-> AMARILLO DE SEMAFORO 1
            GPIO_SetValue(semaforoB.puerto, BITN(semaforoB.pin_amarillo)); //P0.4-> AMARILLO DE SEMAFORO 2

        }

        break;
    }
    if (modo == 0) {
        TIM_UpdateMatchValue(LPC_TIM0, 0, secuencia_Tiempo[secuencia]);
    }
    if (secuencia < 3) {
        secuencia++;
    } else {
        secuencia = 0; //VUELVE AL ESTADO 0  SEMAFORO 1: ROJO SEMAFORO 2: VERDE

        modo = 0; //VUELVE AL ESTADO NORMAL AL TERMINAR LA SECUENCIA PARA EL CIEGO.
    }

    TIM_ResetCounter(LPC_TIM0);
    TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

}

void EINT0_IRQHandler(void) {

    secuencia_Tiempo[0] = 2;
    secuencia_Tiempo[1] = 1;
    secuencia_Tiempo[2] = 2;
    secuencia_Tiempo[3] = 1;

    EXTI_ClearEXTIFlag(0); // limpia bandera
}
void confTIMER1(void) { //ESTE VA A CONTROLAR CADA CUANTO MUESTREA EL ADC
    TIM_MATCHCFG_Type match_cfg; //creo la estructura del Match
    TIM_TIMERCFG_Type timer_cfg; //creo la estructura del timer

    //se puede calcular el timer para 10 min con Prescaler = 6000000, en valor absoluto, no en micro segundos y 10000 el match

    timer_cfg.PrescaleOption = TIM_PRESCALE_USVAL; //1: valor de preescaler en microsegundos (USVAL)
    timer_cfg.PrescaleValue = 6000000; //valor de preescaler dependiendo el modo, en este caso 6 millones de us =6 segundos, supongo que quiere decir que cada 6 segundos se va a incrementar en 1 el timer
    /*---------------------------------------------------------------------------------------------------------------*/
    match_cfg.MatchChannel = 0; //configuro el MATCH0 que se va a habilitar
    match_cfg.IntOnMatch = DISABLE; //cuando matchee que interrumpa el timer.
    match_cfg.StopOnMatch = DISABLE; //cuando matchee que no detenga el timer.
    match_cfg.ResetOnMatch = ENABLE; //cuando matchee que resetee el timer.
    match_cfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING; //no hago nada con el pin de match externo, si no lo tendría que configurar con el pinsel.
    //REVISAR CALCULO
    match_cfg.MatchValue = 2; //cuando el timer llegue a 100 match es decir 600 segundos o 10 minutos.
    /*---------------------------------------------------------------------------------------------------------------*/
    TIM_Init(LPC_TIM1, TIM_TIMER_MODE, & timer_cfg); //Cargo configuracion de TIMER: que timer uso- modo del timer - a que configuracion del timer hago referencia
    TIM_ConfigMatch(LPC_TIM1, & match_cfg); // Cargo configuracion de MATCH que defini
    TIM_Cmd(LPC_TIM1, ENABLE); // activa timer1

    return;
}

void confADC() {

    ADC_Init(LPC_ADC, 200000); //FRECUENCIA DE TRABAJO del ADC. determinar cuando tiene que convertir el ADC
    ADC_BurstCmd(LPC_ADC, 0); //Modo START, NO burst
    ADC_ChannelCmd(LPC_ADC, 0, ENABLE); //activación de canal 0
    ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, SET); //activo interrupción para canal 0
    ADC_EdgeStartConfig(LPC_ADC,ADC_START_ON_RISING);
    // Va a usar el P0.23 como entrada del ADC
    ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT10); // La conversion va a iniciar cuando se produzca el match 0 del timer 1
    LPC_ADC->ADCR |= BITN(26) | BITN(25);

    NVIC_EnableIRQ(ADC_IRQn);//habilitamos interrupciones por adc
}

void ADC_IRQHandler(void) { //cada vez que convierta vamos a decidir cuantos leds de cada luz se deben encender

    static uint16_t ADC0Value = 0; //variable para alamcenar el valor de la conversion del ADC
    ADC0Value = ((LPC_ADC -> ADDR0) >> 4) & 0XFFF;
    //esto es suponiendo que cada luz esta conformada por 3 leds
    if (ADC0Value < 1365) {
        //inidcar que se deben encender los 3 leds
    }
    if (ADC0Value >= 1365 && ADC0Value <= 1365) {
        //indicar que se dene encender 2 leds
    } else {
        //inidicar que se deben encender 1 led
    }
}

void apagarSemaforos(struct Semaforo * semaforoA, struct Semaforo * semaforoB) {
    uint32_t confSemaforoA = 0x0;
    uint32_t confSemaforoB = 0x0;

    confSemaforoA |= BITN(semaforoA -> pin_rojo);
    confSemaforoA |= BITN(semaforoA -> pin_amarillo);
    confSemaforoA |= BITN(semaforoA -> pin_verde);

    confSemaforoB |= BITN(semaforoB -> pin_rojo);
    confSemaforoB |= BITN(semaforoB -> pin_amarillo);
    confSemaforoB |= BITN(semaforoB -> pin_verde);
    GPIO_ClearValue(semaforoA -> puerto, confSemaforoA);
    GPIO_ClearValue(semaforoB -> puerto, confSemaforoB);
}

void confUart(void){
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
    //Habilita interrucpción Tx
    UART_IntConfig(LPC_UART1, UART_INTCFG_THRE, ENABLE);
    // Habilita interrupción por el RX del UART
    UART_IntConfig(LPC_UART1, UART_INTCFG_RBR, ENABLE);
    // Habilita interrupción por el estado de la linea UART
    UART_IntConfig(LPC_UART1, UART_INTCFG_RLS, ENABLE);
    //NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
    //Habilita interrupción por UART1
    NVIC_EnableIRQ(UART1_IRQn);
return;
}

void UART1_IRQHandler(void){
    uint32_t intsrc, tmp, tmp1;
    //Determina la fuente de interrupción
    intsrc = UART_GetIntId(LPC_UART1);
    tmp = intsrc & UART_IIR_INTID_MASK;
    // Evalúa Line Status
    if (tmp == UART_IIR_INTID_RLS){
    tmp1 = UART_GetLineStatus(LPC_UART1);
    tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
    | UART_LSR_BI | UART_LSR_RXFE);
    // ingresa a un Loop infinito si hay error
    if (tmp1) {
        while(1){};
    }
    }
    // Receive Data Available or Character time-out
    if ((tmp == UART_IIR_INTID_RDA)){
        UART_IntReceive();
    }
    if (tmp == UART_IIR_INTID_THRE && sentBufferPtr < BUFFER_LIMIT){
        UART_IntSend();
    }
    return;
}

void UART_IntReceive() {
    uint8_t info[1] = "";
    UART_Receive(LPC_UART1, info, sizeof(info), NONE_BLOCKING);

    if (receivedBufferPtr == BUFFER_LIMIT) {
        receivedBufferPtr = 0;
    }
    if (info[0] == '\r') {
        // El usuario mando un "enter"
        info[0] = '\0';
        receiveBuffer[receivedBufferPtr] = info[0];
        receivedBufferPtr = BUFFER_LIMIT; // Resteo el buffer
    }
    else {
        receiveBuffer[receivedBufferPtr] = info[0];
    }


    if (receivedBufferPtr >= BUFFER_LIMIT) {
        receivedBufferPtr = BUFFER_LIMIT;
    }
    else {
        receivedBufferPtr++;
    }
}
void UART_IntSend() {
    uint32_t buffer_len = 0;
    /*
     * Supongamos sentBufferPtr = 195, BUFFER_LIMIT = 200
     * sentBufferPtr + UART_TX_FIFO_SIZE = 195+16 = 211
     * Tengo que enviar desde la posicion 195 hasta la 200 nomás
     * buffer_len = 200-195 = 5
     */
    if (sentBufferPtr + UART_TX_FIFO_SIZE > BUFFER_LIMIT) {
        buffer_len = BUFFER_LIMIT - sentBufferPtr;
    }
    else buffer_len = UART_TX_FIFO_SIZE;

    uint8_t deberia_terminar = 0;
    for (uint8_t i = sentBufferPtr; i<buffer_len+sentBufferPtr; i++) {
        if (sendBuffer[i] == '\0') {
            buffer_len = i-sentBufferPtr;
            deberia_terminar = 1;
        }
    }
    uint8_t* sendptr =  sendBuffer + sentBufferPtr;
    UART_Send(LPC_UART1, sendptr, buffer_len, NONE_BLOCKING);
    if (deberia_terminar) {
        sentBufferPtr = BUFFER_LIMIT;
    }
    else {
        sentBufferPtr+=buffer_len;
    }

}
