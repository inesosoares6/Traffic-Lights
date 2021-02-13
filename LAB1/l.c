/*
 * LAB1 - Traffic Light controller Part 1
 * MIEEC - SBMI
 *
 *  Created on: 11 Oct 2018
 *  Author: Ines Soares(up201606615) e Silvia Faria(up201603368)
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
 *   Solução: 		Usar um delay para cada estado consoante o tempo que era suposto ficar lá.
 *
 *FUNCIONAMENTO NORMAL
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
#include <serial.h>

#define TGREEN 5000		/*Tempo semaforo verde (50seg)*/
#define TYELLOW 500		/*Tempo semaforo amarelo (5seg)*/
#define TRED 500		/*Tempo semaforo vermelho (5seg)*/
#define TEMG 10000		/*Tempo emergência (10seg)*/
#define TILEGAL 100		/*Tempo amarelo intermitente (1seg)*/

/*Configuração de pinos: Semaforo N-S: PB0-PB2; Semaforo E-O:PB3-PB5*/
#define GNS PB0
#define YNS PB1
#define RNS PB2
#define GEW PB3
#define YEW PB4
#define REW PB5
#define EMG PD2

/*INICIALIZAÇÃO DO SISTEMA (ENTRADAS E SAÍDAS)*/
void hw_init(void){
	DDRB= DDRB | (1<<GNS) | (1<<YNS) | (1<<RNS) | (1<<GEW) | (1<<YEW) | (1<<REW);	/*Pinos PB0-PB5 definidos como saídas*/
	PORTB= PORTB & ~( (1<<GNS) | (1<<YNS) | (1<<RNS) | (1<<GEW) | (1<<YEW) | (1<<REW) ); /*DESLIGA TODOS OS LEDS*/
	DDRD = DDRD & ~(1<<EMG);	/*Pino PD2 definido como entrada*/
}

 /* state: variável da máquina de estados
  * pstate: estado anterior (previous state)
  * nstate: próximo estado (next state)
  * aux: variável que verifica se o estado da máquina já foi escrito para não repetir
 */

int main(void){
	unsigned char state=0, pstate=0, nstate=0, aux=1;
	hw_init(); /*INICIALIZACAO*/

	/*Imprimir na consola*/
	usart_init();
	printf_init();

	if (state!=aux) {
		printf("%d ", state);
		aux=state;
	}
	while(1){

		/*FUNCIONAMENTO NORMAL DOS SEMAFOROS*/
		switch(state){
		case 0: {
			if(PIND&(1<<EMG)){
				pstate=0;
				nstate=13;
			}
			else{
				PORTB=(1<<GNS) | (1<<REW); 	/*Semaforo N-S VERDE		Semaforo E-W VERMELHO*/
				_delay_ms(TGREEN); /*ESPERA 50SEG*/
				nstate=1;
			}
		} break;
		case 1: {
			if(PIND&(1<<EMG)){
				pstate=1;
				nstate=11;
			}
			else{
				PORTB=(1<<YNS) | (1<<REW); 	/*Semaforo N-S AMARELO		Semaforo E-W VERMELHO*/
				_delay_ms(TYELLOW);
				nstate=2;
			}
		} break;
		case 2: {
			if(PIND&(1<<EMG)){
				pstate=2;
				nstate=13;
			}
			else{
				PORTB= (1<<RNS) | (1<<REW);		/*Semaforo N-S VERMELHO		Semaforo E-W VERMELHO*/
				_delay_ms(TRED);
				nstate=3;
			}
		} break;
		case 3: {
			if(PIND&(1<<EMG)){
				pstate=3;
				nstate=13;
			}
			else{
				PORTB=(1<<GEW) | (1<<RNS);	/*Semaforo N-S VERMELHO		Semaforo E-W VERDE*/
				_delay_ms(TGREEN);
				nstate=4;
			}
		} break;
		case 4: {
			if(PIND&(1<<EMG)){
				pstate=4;
				nstate=12;
			}
			else{
				PORTB= (1<<YEW) | (1<<RNS);	/*Semaforo N-S VERMELHO		Semaforo E-W AMARELO*/
				_delay_ms(TYELLOW);
				nstate=5;
			}
		} break;
		case 5: {
			if(PIND&(1<<EMG)){
				pstate=5;
				nstate=13;
			}
			else{
				PORTB= (1<<REW) | (1<<RNS);		/*Semaforo N-S VERMELHO		Semaforo E-W VERMELHO*/
				_delay_ms(TRED);
				nstate=0;
			}
		} break;
		/* CASO DE EMERGENCIA*/
		 case 10: {
			if(0==pstate){
				nstate=11;
			}
			else if(3==pstate){
				nstate=12;
			}
			else if((1==pstate) | (2==pstate) | (4==pstate) | (5==pstate)) {
				nstate=13;
			}
		} break;
		case 11: {
			PORTB= (1<<REW) | (1<<YNS);		/*Semaforo N-S AMARELO		Semaforo E-W VERMELHO*/
			_delay_ms(TYELLOW);
			nstate=13;
		} break;
		case 12: {
			PORTB=(1<<YEW) | (1<<RNS);		/*Semaforo N-S VERMELHO		Semaforo E-W AMARELO*/
			_delay_ms(TYELLOW);
			nstate=13;
		} break;
		case 13: {
			PORTB=(1<<REW) | (1<<RNS);  	/*Semaforo N-S VERMELHO		Semaforo E-W VERMELHO*/
			_delay_ms(TEMG);
			if((0==pstate) & !(PIND&(1<<EMG))){
				nstate=3;
			}
			else if((3==pstate) & !(PIND&(1<<EMG))){
				nstate=0;
			}
			else if((1==pstate) & !(PIND&(1<<EMG))){
				nstate=3;
			}
			else if((4==pstate) & !(PIND&(1<<EMG))){
				nstate=0;
			}
			else if((2==pstate) & !(PIND&(1<<EMG))){
				nstate=3;
			}
			else if((5==pstate) & !(PIND&(1<<EMG))){
				nstate=0;
			}
		}break;
		/*ESTADO ILEGAL - PISCA AMARELO*/
		default : {
			/*PISCAR AMARELO*/
			PORTB=PORTB & ~(1<<RNS) & ~(1<<REW) & ~(1<<GNS) & ~(1<<GEW) & ~(1<<YEW) & ~(1<<YNS);
			while(1){
				PORTB=PORTB ^ (1<<YNS);		/*Semaforo N-S AMARELO*/
				_delay_ms(TILEGAL);
				PORTB=PORTB ^ (1<<YEW);		/*Semaforo E-W AMARELO*/
				_delay_ms(TILEGAL);
			}
		}
	}
		state=nstate;		/*ATUALIZA ESTADO DA MAQUINA*/
	}
}
