/*
 * semaforos.c
 *	LAB2 - Traffic Light controller Part 2
 * 	MIEEC - SBMI
 *
 *  Created on: 18 Oct 2018
 *      Author: Inês Soares(up201606615) e Silvia Faria(up201603368)
 *
 *   Objetivo: 		Desenvolver um programa que controla um cruzamento de semáforos em ruas de sentido único,
 *   			um na direção Norte-Sul (NS) e outro na direção Leste-Oeste (EW).
 *   				Inicialmente o sinal verde na direção NS e o vermelho na direção EW
 *   			devem ficar ligados durante 50 segundos. Seguidamente o sinal verde NS
 *   			deve desligar-se e acender o amarelo NS durante 5 segundos, continuando
 *   			o sinal vermelho EW ligado. Depois de passarem esses 5 segundos,
 *   			o sinal amarelo NS deve desligar-se e ligar-se o vermelho NS durante 5 segundos
 *   			e o sinal vermelho EW continuar ligado. A seguir acontece o descrito em cima,
 *   			mas invertendo o EW e NS. A qualquer momento, se um botão de emergência for ativado,
 *   			o trânsito deve ser interrompido, ficando os dois semáforos vermelhos
 *   			e retomado 10 segundo depois, na direção oposta.
 *					Se for atingido um estado ilegal, as luzes amarelas devem alternar
 *				até que seja redefinido o sistema.
 *
 *   Solução: 		Usar um delay de 10ms no início do ciclo infinito, que torna possível
 *   			a contabilização do tempo, uma vez que é utilizada uma variável (time) que conta
 *   			decrementa sempre que inicia o ciclo e é maior que zero. Quando é igual a zero,
 *   			significa que o tempo definido para aquele estado chegou ao fim e deve transitar
 *   			para o próximo estado.
 *
 * FUNCIONAMENTO NORMAL
 *	State0:
 *		Semaforo N-S verde
 *		Semaforo E-W vermelho
 *		Muda para state1 passado 50seg
 *	State1:
 *		Semaforo N-S amarelo
 *		Semaforo E-W vermelho
 *		Muda para state2 passado 5seg
 *	State2:
 *		Semaforo N-S vermelho
 *		Semaforo E-W vermelho
 *		Muda para state3 passado 5seg
 *	State3:
 *		Semaforo N-S vermelho
 *		Semaforo E-W verde
 *		Muda para state4 passado 50seg
 *	State4:
 *		Semaforo N-S vermelho
 *		Semaforo E-W amarelo
 *		Muda para state5 passado 5seg
 *	State5:
 *		Semaforo N-S vermelho
 *		Semaforo E-W vermelho
 *		Muda para state0 passado 5seg
 *
 *ESTADO DE EMERGÊNCIA
 *	Se botão de emergência for pressionado:
 *		State10:
 *			Se estava em state0 muda para state11 (passa de verde para amarelo antes de passar para vermelho)
 *			Se estava em state3 muda para state12 (passa de verde para amarelo antes de passar para vermelho)
 *			Senão passa diretamente para state13 (se já estava amarelo ou vermelho passa diretamente para vermelho - emergência)
 *		State11:
 *			Semaforo N-S amarelo
 *			Semaforo E-W vermelho
 *			Muda para state13 passado 5seg
 *		State12:
 *			Semaforo N-S vermelho
 *			Semaforo E-W amarelo
 *			Muda para state13 passado 5seg
 *		State13:
 *			Semaforo N-S vermelho
 *			Semaforo E-W vermelho
 *			Passado 10seg retorna ao funcionamento normal mas a circulação em sentido oposto
 *				Se estava nos estados 0, 1 ou 2 muda para state3
 *				Se estava nos estados 3, 4 ou 5 muda para state0
 *
 *ESTADO ILEGAL
 *	Se por algum motivo desconhecido a variavel state adquirir um estado desconhecido passa para state14
 *		State14:
 *			Semaforo N-S amarelo
 *			Semaforo E-W apagado
 *			Muda para state15 passado 1seg
 *		State15:
 *			Semaforo N-S apagado
 *			Semaforo E-W amarelo
 *			Muda para state14 passado 1seg
 *	O sistema só sai do estado ilegal com reset
 */

/*BIBLIOTECAS*/
#include <avr/io.h>
#include <util/delay.h>
#include "serial.h"
#include <avr/interrupt.h>

#define TGREEN 5000		/*Tempo semaforo verde (50seg)*/
#define TYELLOW 500		/*Tempo semaforo amarelo (5seg)*/
#define TRED 500			/*Tempo semaforo vermelho (5seg)*/
#define TEMG 1000		/*Tempo emergência (10seg)*/
#define TILEGAL 100		/*Tempo amarelo intermitente (1seg)*/

/*Configuração de pinos: Semaforo N-S: PB0-PB2; Semaforo E-O:PB3-PB5*/
#define GNS PB0
#define YNS PB1
#define RNS PB2
#define GEW PB3
#define YEW PB4
#define REW PB5
#define EMG PD2 //PD7
#define t 100

/* state: variável da máquina de estados
 * pstate: estado anterior (previous state)
 * nstate: próximo estado (next state)
 * aux: variável que verifica se o estado da máquina já foi escrito para não repetir
 * state_emg: variável que indica se a máquina está ou não em estado de emergência
 * time: variável que conta o tempo
*/
unsigned char  state=0, pstate=0, nstate=0, aux=1, state_emg=0;
uint16_t time;

/*INICIALIZAÇÃO DO SISTEMA (ENTRADAS E SAÍDAS, INTERRUPÇÕES EXTERNAS)*/
void hw_init(void){
	DDRB= DDRB | (1<<GNS) | (1<<YNS) | (1<<RNS) | (1<<GEW) | (1<<YEW) | (1<<REW) ;	/*Pinos PB0-PB5 definidos como saídas*/
	DDRD = DDRD & ~(1<<EMG);														/*Pino PD2 definido como entrada*/
	PORTD = PORTD | (1<<EMG);														/*Ativa pull-up interno*/
	EICRA = EICRA | (2<<ISC00); 													/*Pedido de interrupção em falling edge*/
	EIMSK = EIMSK | (1<<INT0);  													/* Ativa INT0 */

	sei(); 	/*Ativa interrupções globais*/

	time=TGREEN; /*Primeiro tempo: semáforo verde (estado 0)*/

	/*Imprimir na consola*/
	usart_init();
	printf_init();
}

/*INTERRUPÇÃO DA EMERGÊNCIA*/
ISR(INT0_vect)
{
	if(14!=state_emg && 15!=state_emg){
		if(state_emg!=13){
			state=10;
			time=0;
		}
		else{
			state=13;
			time=TEMG;
		}
	}
}

int main(void){

	hw_init(); /*INICIALIZAÇÃO*/

	while(1){

		_delay_ms(t);
		if(time) time--;

		/*IMPRIME NA CONSOLA*/
		if(aux!=state) {
			printf("%u ",state);
			aux=state;
		}

		/*FUNCIONAMENTO NORMAL DOS SEMA�FOROS*/
		switch(state){
		case 0: {
			PORTB= (1<<GNS) | (1<<REW);	/*Semaforo N-S VERDE		Semaforo E-W VERMELHO*/
			pstate=0;
			if(0==time)
			{
				nstate=1;
				time=TYELLOW;
			}
		} break;
		case 1: {
			PORTB= (1<<YNS) | (1<<REW);		/*Semaforo N-S AMARELO		Semaforo E-W VERMELHO*/
			pstate=1;
			if(0==time)
			{
				time=TRED;
				nstate=2;
			}
		} break;
		case 2: {
			PORTB= (1<<RNS) | (1<<REW);		/*Semaforo N-S VERMELHO		Semaforo E-W VERMELHO*/
			pstate=2;
			if(0==time)
			{

				time=TGREEN;
				nstate=3;
			}
		} break;
		case 3: {
			PORTB= (1<<GEW) | (1<<RNS);		/*Semaforo N-S VERMELHO		Semaforo E-W VERDE*/
			pstate=3;
			if(0==time)
			{

				time=TYELLOW;
				nstate=4;
			}

		} break;
		case 4: {
			PORTB= (1<<YEW) | (1<<RNS);		/*Semaforo N-S VERMELHO		Semaforo E-W AMARELO*/
			pstate=4;
			if(0==time)
			{
				time=TRED;
				nstate=5;
			}
		} break;
		case 5: {
			PORTB=(1<<REW) | (1<<RNS);		/*Semaforo N-S VERMELHO		Semaforo E-W VERMELHO*/
			pstate=5;
			if(0==time)
			{
				time=TGREEN;
				nstate=0;
			}
		} break;

		/* CASO DE EMERGENCIA */
		 case 10: {
			state_emg=10;
			if(0==pstate){
				nstate=11;
				time=TYELLOW;
			}
			else if(3==pstate){
				nstate=12;
				time=TYELLOW;
			}
			else{
				nstate=13;
				time=TEMG;
			}
		} break;
		case 11: {
			PORTB= (1<<REW) | (1<<YNS);		/*Semaforo N-S AMARELO		Semaforo E-W VERMELHO*/
			state_emg=11;
			if(0==time){
				time=TEMG;
				nstate=13;
			}
		} break;
		case 12: {
			PORTB= (1<<YEW) | (1<<RNS);		/*Semaforo N-S VERMELHO		Semaforo E-W AMARELO*/
			state_emg=12;
			if(0==time){
				time=TEMG;
				nstate=13;
			}

		} break;
		case 13: {
			PORTB= (1<<REW) | (1<<RNS);		/*Semaforo N-S VERMELHO		Semaforo E-W VERMELHO*/
			state_emg=13;
			if(0==time){
				if(0==pstate|| 1==pstate || 2==pstate){
					nstate=3;
				}
				else{
					nstate=0;

				}
				time=TGREEN;
				state_emg=0;
			}
		}break;
		/*ESTADO ILEGAL - PISCA AMARELO*/
		case 14: {
			PORTB=PORTB & ~(1<<YEW);
			PORTB=PORTB | (1<<YNS);			/*Semaforo N-S AMARELO*/
			state_emg=14;
			if(0==time){
				nstate=15;
				time=TILEGAL;
			}
		}break;
		case 15: {
			PORTB=PORTB & ~(1<<YNS);
			PORTB=PORTB | (1<<YEW);			/*Semaforo E-W AMARELO*/
			state_emg=15;
			if(0==time){
				nstate=14;
				time=TILEGAL;
			}
		}break;

		/*ESTADO ILEGAL - PISCA AMARELO*/
		default : {
			nstate=14;
			PORTB=PORTB & ~(1<<RNS) & ~(1<<REW) & ~(1<<GNS) & ~(1<<GEW) & ~(1<<YEW) & ~(1<<YNS);
			time=TILEGAL;
		}break;
	}
		state=nstate;	/*ATUALIZA ESTADO DA MAQUINA*/
	}
}
